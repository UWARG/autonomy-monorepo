"""Converts AEAC traffic and position commands to MAVROS position setpoints."""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
import rclpy.node
from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.msg import PositionTarget, WaypointAdjustment
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
from obstacle_avoidance.boundary import FlightBoundary
from obstacle_avoidance.waypoint_relocation import outward_replacement
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from navigation.spline_path import SplinePath, aim_point_with_progress
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
    WAYPOINT_ADJUSTMENT_TOPIC = "position_controller/waypoint_adjustment"
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
    STUCK_AFTER_ACTIVE_S = 8.0
    STUCK_PROGRESS_M = 1.5

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
                    map_freshness_s=1.5,
                )
            )
        )
        self._state: State | None = None
        self._target: Coordinate | None = None
        self._path: list[Coordinate] = []
        self._target_index = 0
        self._leg_id = 0
        self._route_index = 0
        self._route_vertices: tuple[Coordinate, ...] = ()
        self._last_adjustment_attempt_sequence = 0
        self._spline: SplinePath | None = None
        self._spline_key: tuple | None = None
        self._aim_arc_m = 0.0
        self._target_received_s = -math.inf
        self._fix: NavSatFix | None = None
        self._fix_received_s = -math.inf
        self._rel_alt_m: float | None = None
        self._rel_alt_received_s = -math.inf
        self._traffic: TrafficSnapshot | None = None
        self._traffic_aircraft: tuple[TrafficAircraft, ...] = ()
        self._traffic_received_s = -math.inf
        self._traffic_timestamp_invalid = False
        self._traffic_motion_error = False
        self._traffic_turns: set[int] = set()
        self._traffic_tracks: dict[int, tuple[float, float, float, float, float]] = {}
        self._traffic_sequence = 0
        self._hold_at: Coordinate | None = None
        self._active = False
        self._blocked_since_s: float | None = None
        self._hold_reason: str | None = None
        self._last_status_s = -math.inf
        self._last_alert_s = -math.inf
        self._progress_anchor: Coordinate | None = None
        self._best_target_distance_m: float | None = None
        self._progress_active_s = 0.0
        self._last_path_s: float | None = None
        self._last_command_heading_rad: float | None = None
        self._tried_headings_rad: list[float] = []
        boundary_raw = os.environ.get("FLIGHT_BOUNDARY_WGS84", "").strip()
        self._boundary: FlightBoundary | None = None
        self._boundary_error = "FLIGHT_BOUNDARY_UNCONFIGURED"
        if boundary_raw:
            try:
                self._boundary = FlightBoundary.from_json(boundary_raw)
                self._boundary_error = ""
            except ValueError:
                self._boundary_error = "INVALID_FLIGHT_BOUNDARY"

        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, self.POSITION_SETPOINT_TOPIC, 10
        )
        self._status_pub = self.create_publisher(
            String, self.AVOIDANCE_STATUS_TOPIC, 10
        )
        self._adjustment_pub = self.create_publisher(
            WaypointAdjustment, self.WAYPOINT_ADJUSTMENT_TOPIC, 10
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
        target = Coordinate(msg.target.lat, msg.target.lon, msg.target.alt)
        if target != self._target or msg.leg_id != self._leg_id:
            self._reset_progress()
        if msg.leg_id != self._leg_id:
            self._last_adjustment_attempt_sequence = 0
        self._leg_id = msg.leg_id
        self._route_index = msg.route_index
        self._route_vertices = tuple(
            Coordinate(point.lat, point.lon, point.alt) for point in msg.route_vertices
        )
        self._target = target
        self._path = [Coordinate(point.lat, point.lon, point.alt) for point in msg.path]
        self._target_index = (
            msg.target_index if msg.target_index < len(self._path) else 0
        )
        if not self._path or self._path[self._target_index] != self._target:
            # A path that does not run through the target is ignored.
            self._path, self._target_index = [], 0
        self._target_received_s = time.monotonic()

    def _traffic_callback(self, msg: TrafficSnapshot) -> None:
        now_s = time.monotonic()
        now_ros_ns = self.get_clock().now().nanoseconds
        source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        source_age_s = (now_ros_ns - source_ns) / 1_000_000_000
        self._traffic_timestamp_invalid = (
            source_ns <= 0 or source_age_s < -0.1 or not math.isfinite(source_age_s)
        )
        if self._traffic_timestamp_invalid:
            source_age_s = 0.0
        self._traffic_motion_error = False
        self._traffic_turns = set()
        if (
            msg.sequence
            and self._traffic_sequence
            and msg.sequence <= self._traffic_sequence
        ):
            self._traffic_motion_error = True
        self._traffic_sequence = msg.sequence
        sample_s = now_s - max(0.0, source_age_s)
        tracks: dict[int, tuple[float, float, float, float, float]] = {}
        if msg.connected and msg.healthy:
            for item in msg.aircraft:
                previous = self._traffic_tracks.get(item.aircraft_index)
                if previous is not None:
                    elapsed_s = sample_s - previous[0]
                    if elapsed_s <= 0.0:
                        self._traffic_motion_error = True
                    else:
                        east_m, north_m = east_north_coordinate_offset_m(
                            previous[1],
                            previous[2],
                            item.latitude_deg,
                            item.longitude_deg,
                        )
                        apparent_speed = math.hypot(east_m, north_m) / elapsed_s
                        allowed_speed = max(
                            0.75, 1.5 * max(previous[3], item.speed_mps) + 0.75
                        )
                        if (
                            not math.isfinite(apparent_speed)
                            or apparent_speed > allowed_speed
                        ):
                            self._traffic_motion_error = True
                        heading_change = abs(
                            (item.heading_deg_true - previous[4] + 180.0) % 360.0
                            - 180.0
                        )
                        if heading_change >= 60.0:
                            self._traffic_turns.add(item.aircraft_index)
                        if math.hypot(east_m, north_m) >= 2.0:
                            course_deg = (
                                math.degrees(math.atan2(east_m, north_m)) % 360.0
                            )
                            course_error = abs(
                                (course_deg - item.heading_deg_true + 180.0) % 360.0
                                - 180.0
                            )
                            if course_error >= 60.0:
                                self._traffic_turns.add(item.aircraft_index)
                                self._traffic_motion_error = True
                tracks[item.aircraft_index] = (
                    sample_s,
                    item.latitude_deg,
                    item.longitude_deg,
                    item.speed_mps,
                    item.heading_deg_true,
                )
        self._traffic_tracks = tracks
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
        self._traffic_received_s = sample_s

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
        if self._boundary is None:
            return self._hold(position, self._boundary_error, now_s)
        if not self._boundary.allows_point(position):
            return self._hold(position, "FLIGHT_BOUNDARY_POSITION_UNSAFE", now_s)
        if not self._boundary.allows_point(self._target):
            return self._hold(position, "TARGET_OUTSIDE_FLIGHT_BOUNDARY", now_s)
        traffic = self._traffic
        if traffic is None:
            return self._hold(position, "NO_TRAFFIC", now_s)
        if not traffic.connected:
            return self._hold(position, traffic.reason or "TRAFFIC_DISCONNECTED", now_s)
        if not traffic.healthy:
            return self._hold(position, traffic.reason or "UNHEALTHY_TRAFFIC", now_s)
        if self._traffic_timestamp_invalid:
            return self._hold(position, "INVALID_TRAFFIC_RECEIVE_TIME", now_s)
        if self._traffic_motion_error:
            return self._hold(position, "INCONSISTENT_TRAFFIC_MOTION", now_s)
        if not 0 <= traffic.own_aircraft_index <= 255:
            return self._hold(position, "OWN_AIRCRAFT_INDEX_UNCONFIRMED", now_s)
        if any(
            aircraft.aircraft_index == traffic.own_aircraft_index
            for aircraft in self._traffic_aircraft
        ):
            return self._hold(position, "OWN_AIRCRAFT_IN_TRAFFIC", now_s)

        aim = self._aim_point(position)
        self._update_progress(position, now_s)
        traffic_age_s = now_s - self._traffic_received_s
        plan = self._avoider.plan(
            AvoidanceRequest(
                position=position,
                goal=aim,
                traffic=self._traffic_aircraft,
                traffic_age_s=traffic_age_s,
                excluded_headings_rad=tuple(self._tried_headings_rad),
                required_waypoint=self._target,
            )
        )
        if plan.reason == "AIM_IN_TRAFFIC_BUFFER" and aim != self._target:
            # A blocked look-ahead point is optional; the current mission
            # waypoint is not. Replan to that waypoint without skipping it.
            aim = self._target
            plan = self._avoider.plan(
                AvoidanceRequest(
                    position=position,
                    goal=aim,
                    traffic=self._traffic_aircraft,
                    traffic_age_s=traffic_age_s,
                    excluded_headings_rad=tuple(self._tried_headings_rad),
                    required_waypoint=self._target,
                )
            )
        if plan.status is not PlanStatus.PATH_FOUND or plan.next_waypoint is None:
            reason = plan.reason or "NO_PATH"
            substituted = (
                0 <= self._route_index < len(self._route_vertices)
                and self._target != self._route_vertices[self._route_index]
            )
            if reason == "WAYPOINT_IN_TRAFFIC_BUFFER" or (
                substituted and reason in {"BLOCKED", "AIM_IN_TRAFFIC_BUFFER"}
            ):
                self._propose_replacement(position, traffic_age_s)
            if self._tried_headings_rad and reason == "BLOCKED":
                reason = "STUCK_NO_SAFE_ALTERNATE"
            return self._hold(position, reason, now_s)

        next_setpoint = aim if plan.goal_reached else plan.next_waypoint
        if not self._boundary.allows_segment(position, next_setpoint):
            return self._hold(position, "SETPOINT_OUTSIDE_FLIGHT_BOUNDARY", now_s)
        east_m, north_m = east_north_coordinate_offset_m(
            position.lat,
            position.lon,
            next_setpoint.lat,
            next_setpoint.lon,
        )
        if math.hypot(east_m, north_m) > 0.1:
            self._last_command_heading_rad = math.atan2(north_m, east_m)
        self._last_path_s = now_s

        self._hold_at = None
        self._blocked_since_s = None
        self._hold_reason = None
        self._report_status(now_s, "PATH_FOUND", plan.minimum_clearance_m)
        return next_setpoint

    def _propose_replacement(self, position: Coordinate, traffic_age_s: float) -> None:
        if (
            self._boundary is None
            or self._leg_id == 0
            or self._traffic_sequence == 0
            or self._traffic_sequence <= self._last_adjustment_attempt_sequence
            or not 0 <= self._route_index < len(self._route_vertices)
        ):
            return
        self._last_adjustment_attempt_sequence = self._traffic_sequence
        original = self._route_vertices[self._route_index]
        current = self._target
        if current is None:
            return
        current_e, current_n = east_north_coordinate_offset_m(
            original.lat, original.lon, current.lat, current.lon
        )
        shift_m = math.hypot(current_e, current_n)
        replacement = outward_replacement(
            position=position,
            original=original,
            route=self._route_vertices,
            route_index=self._route_index,
            traffic=self._traffic_aircraft,
            age_s=traffic_age_s,
            boundary=self._boundary,
            avoider=self._avoider,
            min_shift_m=shift_m,
        )
        if replacement is None:
            return
        self._adjustment_pub.publish(
            WaypointAdjustment(
                leg_id=self._leg_id,
                route_index=self._route_index,
                original=CoordinateMsg(
                    lat=original.lat, lon=original.lon, alt=original.alt
                ),
                replacement=CoordinateMsg(
                    lat=replacement.lat, lon=replacement.lon, alt=replacement.alt
                ),
                traffic_sequence=self._traffic_sequence,
            )
        )

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
            self._aim_arc_m = 0.0
        (east_m, north_m, alt_m), self._aim_arc_m = aim_point_with_progress(
            self._spline,
            self._target_index,
            local(position),
            self.PATH_LOOKAHEAD_M,
            self.CORNER_LOOKAHEAD_M,
            self._aim_arc_m,
        )
        return enu_to_coordinate(target, east_m, north_m, alt_m)

    def _reset_progress(self) -> None:
        self._progress_anchor = None
        self._best_target_distance_m = None
        self._progress_active_s = 0.0
        self._last_path_s = None
        self._last_command_heading_rad = None
        self._tried_headings_rad = []

    def _update_progress(self, position: Coordinate, now_s: float) -> None:
        to_target_east_m, to_target_north_m = east_north_coordinate_offset_m(
            position.lat,
            position.lon,
            self._target.lat,
            self._target.lon,
        )
        target_distance_m = math.hypot(to_target_east_m, to_target_north_m)
        if self._best_target_distance_m is None:
            self._best_target_distance_m = target_distance_m
        elif target_distance_m <= self._best_target_distance_m - self.STUCK_PROGRESS_M:
            # Lateral motion can reset the stall timer, but must not make a
            # failed heading eligible again while circling the same waypoint.
            self._best_target_distance_m = target_distance_m
            self._tried_headings_rad.clear()
        if self._progress_anchor is None:
            self._progress_anchor = position
            return
        east_m, north_m = east_north_coordinate_offset_m(
            self._progress_anchor.lat,
            self._progress_anchor.lon,
            position.lat,
            position.lon,
        )
        if math.hypot(east_m, north_m) >= self.STUCK_PROGRESS_M:
            self._progress_anchor = position
            self._progress_active_s = 0.0
            self._last_path_s = None
            return
        if self._last_path_s is not None and 0 <= now_s - self._last_path_s <= 0.5:
            self._progress_active_s += now_s - self._last_path_s
        if (
            self._progress_active_s >= self.STUCK_AFTER_ACTIVE_S
            and self._last_command_heading_rad is not None
        ):
            self._tried_headings_rad.append(self._last_command_heading_rad)
            self._progress_active_s = 0.0
            self._avoider.reset()

    def _hold(
        self, position: Coordinate | None, reason: str, now_s: float
    ) -> Coordinate | None:
        """Latch the position where the hold began and keep returning it."""

        self._avoider.reset()
        self._last_path_s = None
        if reason in (
            "BLOCKED",
            "AIM_IN_TRAFFIC_BUFFER",
            "WAYPOINT_IN_TRAFFIC_BUFFER",
            "YIELD_CROSSING",
            "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED",
        ):
            if self._blocked_since_s is None or self._hold_reason != reason:
                self._blocked_since_s = now_s
        else:
            self._blocked_since_s = None
        self._hold_reason = reason
        alert_after_s = 20.0 if reason == "YIELD_CROSSING" else 5.0
        urgent = reason in {
            "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED",
            "STALE_TRAFFIC",
            "TRAFFIC_SPEED_OUT_OF_ENVELOPE",
            "INCONSISTENT_TRAFFIC_MOTION",
            "INVALID_TRAFFIC_RECEIVE_TIME",
            "AEAC_DISCONNECTED",
            "FLIGHT_BOUNDARY_UNCONFIGURED",
            "INVALID_FLIGHT_BOUNDARY",
            "FLIGHT_BOUNDARY_POSITION_UNSAFE",
            "TARGET_OUTSIDE_FLIGHT_BOUNDARY",
            "SETPOINT_OUTSIDE_FLIGHT_BOUNDARY",
            "STUCK_NO_SAFE_ALTERNATE",
        }
        takeover = urgent or (
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
        self._reset_progress()
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
                        + live_traffic_buffer_m(
                            aircraft.speed_mps,
                            age_s,
                            response_s=self._avoider.config.live_response_s,
                        )
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
            "hold_reason": self._hold_reason,
            "traffic_age_s": age_s if math.isfinite(age_s) else None,
            "minimum_clearance_m": minimum_clearance_m,
            "radii": radii,
            "sharp_turn_indices": sorted(self._traffic_turns),
            "tried_heading_count": len(self._tried_headings_rad),
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
