"""Converts AEAC traffic and position commands to MAVROS position setpoints."""

from __future__ import annotations

import json
import math
import time

import rclpy
import rclpy.node
from airside_interfaces.msg import PositionTarget
from airside_interfaces.msg import TrafficSnapshot
from mavros_msgs.msg import GlobalPositionTarget, State, StatusText
from obstacle_avoidance import (
    AvoidanceConfig,
    AvoidanceRequest,
    ObstacleAvoider,
    PlannerConfig,
    PlanStatus,
    TrafficAircraft,
    live_traffic_buffer_m,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from navigation.spline_path import SplinePath, aim_point
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m


class PositionController(rclpy.node.Node):
    """
    Takes the engine's position target, runs obstacle avoidance against traffic,
    and publishes the result as a MAVROS guided position setpoint.

    The setpoint is the avoider's next safe point, or the target itself once
    within tolerance. If the target comes with a path through it, the drone
    instead aims ahead along a smooth spline through the path. With no safe
    path, the avoider holds and replans on fresh traffic. If traffic is
    predicted to cross the hold point, the pilot must take over. Anything
    that makes avoidance impossible (no or stale traffic, stale position, no
    safe path) holds where the drone was when the hold began. Nothing is
    published unless armed in GUIDED. If the engine stops sending targets,
    it holds once where the drone is, then lets go.
    """

    TARGET_TOPIC = "position_controller/target"
    TRAFFIC_TOPIC = "/aeac/traffic"
    STATE_TOPIC = "mavros/state"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
    REL_ALT_TOPIC = "mavros/global_position/rel_alt"
    POSITION_SETPOINT_TOPIC = "mavros/setpoint_raw/global"
    AVOIDANCE_STATUS_TOPIC = "position_controller/avoidance_status"
    PILOT_ALERT_TOPIC = "mavros/statustext/send"
    GUIDED_MODE = "GUIDED"
    CONTROL_RATE_HZ = 10.0
    TARGET_TIMEOUT_S = 1.5
    # How far ahead of the drone, along the path, to aim on straights.
    PATH_LOOKAHEAD_M = 16.0
    # Approaching a waypoint, the lookahead shrinks to keep aiming at it, down
    # to this. Bounds how far inside the spline the drone cuts a corner.
    CORNER_LOOKAHEAD_M = 3.0
    POSITION_FRESHNESS_S = 1.0

    TYPE_MASK = (
        GlobalPositionTarget.IGNORE_VX
        | GlobalPositionTarget.IGNORE_VY
        | GlobalPositionTarget.IGNORE_VZ
        | GlobalPositionTarget.IGNORE_AFX
        | GlobalPositionTarget.IGNORE_AFY
        | GlobalPositionTarget.IGNORE_AFZ
        | GlobalPositionTarget.IGNORE_YAW
        | GlobalPositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self) -> None:
        super().__init__("position_controller")
        self._avoider = ObstacleAvoider(
            AvoidanceConfig(
                planner=PlannerConfig(
                    first_lookahead_m=self.PATH_LOOKAHEAD_M,
                    map_freshness_s=2.5,
                )
            )
        )
        self._state: State | None = None
        self._target: Coordinate | None = None
        self._path: list[Coordinate] = []
        self._target_index = 0
        self._spline: SplinePath | None = None
        self._spline_key: tuple | None = None
        self._target_received_s = -math.inf
        self._fix: NavSatFix | None = None
        self._fix_received_s = -math.inf
        self._rel_alt_m: float | None = None
        self._rel_alt_received_s = -math.inf
        self._traffic: TrafficSnapshot | None = None
        self._traffic_aircraft: tuple[TrafficAircraft, ...] = ()
        self._traffic_received_s = -math.inf
        self._hold_at: Coordinate | None = None
        self._active = False
        self._blocked_since_s: float | None = None
        self._hold_reason: str | None = None
        self._last_status_s = -math.inf
        self._last_alert_s = -math.inf

        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, self.POSITION_SETPOINT_TOPIC, 10
        )
        self._status_pub = self.create_publisher(
            String, self.AVOIDANCE_STATUS_TOPIC, 10
        )
        self._pilot_alert_pub = self.create_publisher(
            StatusText, self.PILOT_ALERT_TOPIC, 10
        )
        self.create_subscription(
            PositionTarget, self.TARGET_TOPIC, self._target_callback, 10
        )
        self.create_subscription(
            TrafficSnapshot, self.TRAFFIC_TOPIC, self._traffic_callback, 10
        )
        self.create_subscription(State, self.STATE_TOPIC, self._state_callback, 10)
        self.create_subscription(
            NavSatFix,
            self.GLOBAL_POSITION_TOPIC,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64, self.REL_ALT_TOPIC, self._rel_alt_callback, qos_profile_sensor_data
        )
        self.create_timer(1.0 / self.CONTROL_RATE_HZ, self._control_cycle)

        self.get_logger().info(
            "Position controller ready - sole MAVROS position setpoint owner, "
            f"avoiding traffic on '{self.TRAFFIC_TOPIC}'."
        )

    def _target_callback(self, msg: PositionTarget) -> None:
        self._target = Coordinate(msg.target.lat, msg.target.lon, msg.target.alt)
        self._path = [Coordinate(point.lat, point.lon, point.alt) for point in msg.path]
        self._target_index = msg.target_index if msg.target_index < len(self._path) else 0
        if not self._path or self._path[self._target_index] != self._target:
            # A path that does not run through the target is ignored.
            self._path, self._target_index = [], 0
        self._target_received_s = time.monotonic()

    def _traffic_callback(self, msg: TrafficSnapshot) -> None:
        self._traffic = msg
        self._traffic_aircraft = tuple(
            TrafficAircraft(
                aircraft_index=item.aircraft_index,
                lat=item.latitude_deg,
                lon=item.longitude_deg,
                altitude_agl_m=item.altitude_agl_m,
                speed_mps=item.speed_mps,
                heading_deg=item.heading_deg_true,
                horizontal_keep_away_m=item.horizontal_keepaway_m,
                vertical_keep_away_m=item.vertical_keepaway_m,
            )
            for item in msg.aircraft
        )
        self._traffic_received_s = time.monotonic()

    def _state_callback(self, msg: State) -> None:
        self._state = msg

    def _fix_callback(self, msg: NavSatFix) -> None:
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self._fix = msg
            self._fix_received_s = time.monotonic()

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._rel_alt_m = msg.data
        self._rel_alt_received_s = time.monotonic()

    def _command_allowed(self) -> bool:
        return bool(
            self._state is not None
            and self._state.connected
            and self._state.armed
            and self._state.mode == self.GUIDED_MODE
        )

    def _position(self) -> Coordinate | None:
        if self._fix is None or self._rel_alt_m is None:
            return None
        return Coordinate(self._fix.latitude, self._fix.longitude, self._rel_alt_m)

    def _control_cycle(self) -> None:
        setpoint = self._next_setpoint(time.monotonic())
        if setpoint is not None:
            self._publish_setpoint(setpoint)

    def _next_setpoint(self, now_s: float) -> Coordinate | None:
        """Where to send the drone this cycle, or None to publish nothing."""

        if not self._command_allowed():
            self._release()
            return None
        position = self._position()
        if (
            self._target is None
            or now_s - self._target_received_s > self.TARGET_TIMEOUT_S
        ):
            # The engine stopped asking: stop where we are, once, then let go.
            hold = self._hold(position, "NO_TARGET", now_s) if self._active else None
            self._release()
            return hold

        self._active = True
        if position is None:
            return self._hold(None, "NO_POSITION", now_s)
        position_received_s = min(self._fix_received_s, self._rel_alt_received_s)
        if not 0.0 <= now_s - position_received_s <= self.POSITION_FRESHNESS_S:
            return self._hold(position, "STALE_POSITION", now_s)
        traffic = self._traffic
        if traffic is None:
            return self._hold(position, "NO_TRAFFIC", now_s)
        if not traffic.connected:
            return self._hold(position, traffic.reason or "TRAFFIC_DISCONNECTED", now_s)
        if not traffic.healthy:
            return self._hold(position, traffic.reason or "UNHEALTHY_TRAFFIC", now_s)
        if not 0 <= traffic.own_aircraft_index <= 255:
            return self._hold(position, "OWN_AIRCRAFT_INDEX_UNCONFIRMED", now_s)
        if any(
            aircraft.aircraft_index == traffic.own_aircraft_index
            for aircraft in self._traffic_aircraft
        ):
            return self._hold(position, "OWN_AIRCRAFT_IN_TRAFFIC", now_s)

        aim = self._aim_point(position)
        plan = self._avoider.plan(
            AvoidanceRequest(
                position=position,
                goal=aim,
                traffic=self._traffic_aircraft,
                traffic_age_s=now_s - self._traffic_received_s,
            )
        )
        if plan.status is not PlanStatus.PATH_FOUND or plan.next_waypoint is None:
            return self._hold(position, plan.reason or "NO_PATH", now_s)

        self._hold_at = None
        self._blocked_since_s = None
        self._hold_reason = None
        self._report_status(now_s, "PATH_FOUND", plan.minimum_clearance_m)
        return aim if plan.goal_reached else plan.next_waypoint

    def _aim_point(self, position: Coordinate) -> Coordinate:
        """
        The target, or with a path through it, the point ahead of the drone
        along the spline through the path.
        """

        target = self._target
        if len(self._path) < 2:
            return target

        def local(point: Coordinate) -> tuple[float, float, float]:
            east_m, north_m = east_north_coordinate_offset_m(
                target.lat, target.lon, point.lat, point.lon
            )
            return (east_m, north_m, point.alt)

        key = (tuple(self._path), self._target_index)
        if key != self._spline_key:
            self._spline = SplinePath([local(point) for point in self._path])
            self._spline_key = key
        east_m, north_m, alt_m = aim_point(
            self._spline,
            self._target_index,
            local(position),
            self.PATH_LOOKAHEAD_M,
            self.CORNER_LOOKAHEAD_M,
        )
        return enu_to_coordinate(target, east_m, north_m, alt_m)

    def _hold(
        self, position: Coordinate | None, reason: str, now_s: float
    ) -> Coordinate | None:
        """Latch the position where the hold began and keep returning it."""

        self._avoider.reset()
        if reason in (
            "BLOCKED",
            "YIELD_CROSSING",
            "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED",
        ):
            if self._blocked_since_s is None or self._hold_reason != reason:
                self._blocked_since_s = now_s
        else:
            self._blocked_since_s = None
        self._hold_reason = reason
        alert_after_s = 20.0 if reason == "YIELD_CROSSING" else 5.0
        takeover = reason == "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED" or (
            self._blocked_since_s is not None
            and now_s - self._blocked_since_s >= alert_after_s
        )
        if takeover and now_s - self._last_alert_s >= 5.0:
            alert = StatusText()
            alert.header.stamp = self.get_clock().now().to_msg()
            alert.severity = StatusText.WARNING
            alert.text = "AEAC no safe path - pilot take over"
            self._pilot_alert_pub.publish(alert)
            self.get_logger().error(alert.text)
            self._last_alert_s = now_s
        if self._hold_at is None:
            self._hold_at = position
        self.get_logger().warning(
            f"Holding position: {reason}", throttle_duration_sec=2.0
        )
        self._report_status(
            now_s, "PILOT_TAKEOVER_REQUIRED" if takeover else reason, None
        )
        # With no known position, publish nothing; the FC keeps its last setpoint.
        return self._hold_at

    def _release(self) -> None:
        self._avoider.reset()
        self._hold_at = None
        self._active = False
        self._blocked_since_s = None
        self._hold_reason = None

    def _report_status(
        self, now_s: float, reason: str, minimum_clearance_m: float | None
    ) -> None:
        """Publish token-free, per-aircraft radii and hold reasons at 1 Hz."""

        if now_s - self._last_status_s < 1.0:
            return
        age_s = now_s - self._traffic_received_s
        radii = []
        if math.isfinite(age_s) and age_s >= 0.0:
            for aircraft in self._traffic_aircraft:
                try:
                    expanded_m = (
                        aircraft.horizontal_keep_away_m
                        + live_traffic_buffer_m(aircraft.speed_mps, age_s)
                    )
                except ValueError:
                    continue
                radii.append(
                    {
                        "aircraft_index": aircraft.aircraft_index,
                        "raw_radius_m": aircraft.horizontal_keep_away_m,
                        "expanded_radius_m": expanded_m,
                    }
                )
        payload = {
            "reason": reason,
            "traffic_age_s": age_s if math.isfinite(age_s) else None,
            "minimum_clearance_m": minimum_clearance_m,
            "radii": radii,
        }
        self._status_pub.publish(String(data=json.dumps(payload)))
        self._last_status_s = now_s

    def _publish_setpoint(self, target: Coordinate) -> None:
        setpoint = GlobalPositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        setpoint.type_mask = self.TYPE_MASK
        setpoint.latitude = target.lat
        setpoint.longitude = target.lon
        setpoint.altitude = target.alt
        self._setpoint_pub.publish(setpoint)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
