"""Converts AEAC traffic and position commands to MAVROS position setpoints."""

from __future__ import annotations

import math
import time

import rclpy
import rclpy.node
from airside_interfaces.msg import PositionTarget
from airside_interfaces.msg import TrafficSnapshot
from mavros_msgs.msg import GlobalPositionTarget, State
from obstacle_avoidance import (
    ESCAPE_REASON,
    AvoidanceRequest,
    ObstacleAvoider,
    PlanStatus,
    TrafficAircraft,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m


class PositionController(rclpy.node.Node):
    """
    Takes the engine's position target, runs gnc obstacle avoidance against
    the latest AEAC traffic, and publishes the result as a MAVROS position
    setpoint at 10 Hz.

    The setpoint is the avoider's next safe point, or the target itself once
    within tolerance. If the target comes with a following waypoint, the aim
    point slides past the target onto the next leg once the drone is within
    ``CORNER_LOOKAHEAD_M``, so the drone rounds the corner instead of
    stopping. With no path to the target (e.g. traffic flying at us),
    the avoider backs away to the clearest reachable point rather than hold. Anything that makes avoidance impossible (no or stale
    traffic, stale position, no safe path) holds where the drone was when the
    hold began. Nothing is published unless armed in GUIDED. If the engine
    stops sending targets, it holds once where the drone is, then lets go.
    """

    TARGET_TOPIC = "position_controller/target"
    TRAFFIC_TOPIC = "/aeac/traffic"
    STATE_TOPIC = "mavros/state"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
    REL_ALT_TOPIC = "mavros/global_position/rel_alt"
    POSITION_SETPOINT_TOPIC = "mavros/setpoint_raw/global"
    GUIDED_MODE = "GUIDED"
    CONTROL_RATE_HZ = 10.0
    # The engine republishes its target every tick.
    TARGET_TIMEOUT_S = 1.5
    # How far ahead of the drone, along the path, to aim when rounding a
    # corner. Larger is smoother and faster but cuts the corner wider.
    CORNER_LOOKAHEAD_M = 5.0
    POSITION_FRESHNESS_S = 1.0

    # Position-only setpoint: ignores velocity, acceleration and yaw fields.
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
        self._avoider = ObstacleAvoider()
        self._state: State | None = None
        self._target: Coordinate | None = None
        self._following: Coordinate | None = None
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

        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, self.POSITION_SETPOINT_TOPIC, 10
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
        self._following = (
            Coordinate(msg.following[0].lat, msg.following[0].lon, msg.following[0].alt)
            if msg.following
            else None
        )
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
        if self._target is None or now_s - self._target_received_s > self.TARGET_TIMEOUT_S:
            # The engine stopped asking: stop where we are, once, then let go.
            hold = self._hold(position, "NO_TARGET") if self._active else None
            self._release()
            return hold

        self._active = True
        if position is None:
            return self._hold(None, "NO_POSITION")
        position_received_s = min(self._fix_received_s, self._rel_alt_received_s)
        if not 0.0 <= now_s - position_received_s <= self.POSITION_FRESHNESS_S:
            return self._hold(position, "STALE_POSITION")
        traffic = self._traffic
        if traffic is None:
            return self._hold(position, "NO_TRAFFIC")
        if not traffic.connected:
            return self._hold(position, traffic.reason or "TRAFFIC_DISCONNECTED")
        if not traffic.healthy:
            return self._hold(position, traffic.reason or "UNHEALTHY_TRAFFIC")

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
            return self._hold(position, plan.reason or "NO_PATH")

        self._hold_at = None
        if plan.reason == ESCAPE_REASON:
            self.get_logger().warning(
                "No path to the target: backing away from traffic",
                throttle_duration_sec=2.0,
            )
        return aim if plan.goal_reached else plan.next_waypoint

    def _aim_point(self, position: Coordinate) -> Coordinate:
        """
        The target, or with a following waypoint and the drone within
        ``CORNER_LOOKAHEAD_M`` of the target, the point the leftover lookahead
        distance along the next leg.
        """

        target, following = self._target, self._following
        if following is None:
            return target
        to_target_m = math.hypot(
            *east_north_coordinate_offset_m(
                position.lat, position.lon, target.lat, target.lon
            )
        )
        if to_target_m >= self.CORNER_LOOKAHEAD_M:
            return target
        leg_east_m, leg_north_m = east_north_coordinate_offset_m(
            target.lat, target.lon, following.lat, following.lon
        )
        leg_m = math.hypot(leg_east_m, leg_north_m)
        if leg_m == 0.0:
            return target
        fraction = min(self.CORNER_LOOKAHEAD_M - to_target_m, leg_m) / leg_m
        return enu_to_coordinate(
            target,
            leg_east_m * fraction,
            leg_north_m * fraction,
            target.alt + (following.alt - target.alt) * fraction,
        )

    def _hold(self, position: Coordinate | None, reason: str) -> Coordinate | None:
        """Latch the position where the hold began and keep returning it."""

        self._avoider.reset()
        if self._hold_at is None:
            self._hold_at = position
        self.get_logger().warning(
            f"Holding position: {reason}", throttle_duration_sec=2.0
        )
        # With no known position, publish nothing; the FC keeps its last setpoint.
        return self._hold_at

    def _release(self) -> None:
        self._avoider.reset()
        self._hold_at = None
        self._active = False

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
