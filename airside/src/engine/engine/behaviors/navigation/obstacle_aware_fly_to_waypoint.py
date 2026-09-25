"""Obstacle-aware lapping navigation using GUIDED velocity setpoints."""

from __future__ import annotations

import math
import time

import py_trees
import rclpy.node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from obstacle_avoidance import PlannerConfig, Point2D, sector_scan_to_snapshot
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from engine import blackboard_keys
from engine.constants import (
    GUIDED_MODE,
    WAYPOINT_ACCEPTANCE_RADIUS_M,
    WAYPOINT_NAV_TIMEOUT_S,
)
from engine.obstacle_navigation import (
    ActiveNavigationClock,
    NavigationDecision,
    NavigationGoal,
    NavigationTelemetry,
    ObstacleAwareController,
    ObstacleNavigationConfig,
    ScanConversion,
    prepare_sector_scan,
)
from utils.src.waypoint_utils import east_north_coordinate_offset_m

_SCAN_TOPIC_DEFAULT = "/obstacle_avoidance/scan"
_DIAGNOSTICS_TOPIC = "/obstacle_avoidance/diagnostics"
_VELOCITY_TOPIC = "mavros/setpoint_velocity/cmd_vel"
_LOCAL_POSE_TOPIC = "mavros/local_position/pose"
_GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_STATE_TOPIC = "mavros/state"


class ObstacleAwareFlyToWaypoint(py_trees.behaviour.Behaviour):
    """Fly to ``current_waypoint`` while the 2D planner owns horizontal motion."""

    def __init__(self, name: str = "ObstacleAwareFlyToWaypoint") -> None:
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT,
            access=py_trees.common.Access.READ,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._declare_parameters()

        self._config = ObstacleNavigationConfig(
            control_rate_hz=self._parameter_float("obstacle_avoidance.control_rate_hz"),
            horizontal_speed_mps=self._parameter_float(
                "obstacle_avoidance.horizontal_speed_mps"
            ),
            vertical_speed_mps=self._parameter_float(
                "obstacle_avoidance.vertical_speed_mps"
            ),
            obstacle_radius_m=self._parameter_float(
                "obstacle_avoidance.obstacle_radius_m"
            ),
            scan_freshness_s=self._parameter_float(
                "obstacle_avoidance.scan_freshness_s"
            ),
            telemetry_freshness_s=self._parameter_float(
                "obstacle_avoidance.telemetry_freshness_s"
            ),
            goal_tolerance_m=WAYPOINT_ACCEPTANCE_RADIUS_M,
            expected_scan_frame=self._parameter_string(
                "obstacle_avoidance.scan_frame"
            ),
            guided_mode=GUIDED_MODE,
        )
        planner_config = PlannerConfig(
            first_lookahead_m=self._parameter_float(
                "obstacle_avoidance.first_lookahead_m"
            ),
            second_lookahead_m=self._parameter_float(
                "obstacle_avoidance.second_lookahead_m"
            ),
            clearance_margin_m=self._parameter_float(
                "obstacle_avoidance.clearance_margin_m"
            ),
            map_freshness_s=self._config.scan_freshness_s,
            hysteresis_cost_m=self._parameter_float(
                "obstacle_avoidance.hysteresis_cost_m"
            ),
        )
        self._controller = ObstacleAwareController(self._config, planner_config)
        self._navigation_clock = ActiveNavigationClock()

        self._latest_pose: PoseStamped | None = None
        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._latest_scan: ScanConversion | None = None
        self._last_scan_source_stamp_s: float | None = None
        self._pose_received_s: float | None = None
        self._fix_received_s: float | None = None
        self._altitude_received_s: float | None = None
        self._state_received_s: float | None = None
        self._guided_entry_s: float | None = None
        self._waypoint = None
        self._active = False
        self._timed_out = False
        self._latest_decision: NavigationDecision | None = None

        self._velocity_pub = self._node.create_publisher(
            TwistStamped, _VELOCITY_TOPIC, 10
        )
        self._diagnostics_pub = self._node.create_publisher(
            DiagnosticArray, _DIAGNOSTICS_TOPIC, 10
        )
        self._pose_sub = self._node.create_subscription(
            PoseStamped,
            _LOCAL_POSE_TOPIC,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self._fix_sub = self._node.create_subscription(
            NavSatFix,
            _GLOBAL_POSITION_TOPIC,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self._rel_alt_sub = self._node.create_subscription(
            Float64,
            _REL_ALT_TOPIC,
            self._rel_alt_callback,
            qos_profile_sensor_data,
        )
        self._state_sub = self._node.create_subscription(
            State, _STATE_TOPIC, self._state_callback, 10
        )
        self._scan_sub = self._node.create_subscription(
            LaserScan,
            self._parameter_string("obstacle_avoidance.scan_topic"),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._control_timer = self._node.create_timer(
            1.0 / self._config.control_rate_hz,
            self._control_cycle,
        )

    def _declare_parameters(self) -> None:
        defaults: dict[str, object] = {
            "obstacle_avoidance.scan_topic": _SCAN_TOPIC_DEFAULT,
            "obstacle_avoidance.scan_frame": "base_link",
            "obstacle_avoidance.control_rate_hz": 10.0,
            "obstacle_avoidance.horizontal_speed_mps": 2.0,
            "obstacle_avoidance.vertical_speed_mps": 1.0,
            "obstacle_avoidance.first_lookahead_m": 8.0,
            "obstacle_avoidance.second_lookahead_m": 8.0,
            "obstacle_avoidance.clearance_margin_m": 1.0,
            "obstacle_avoidance.obstacle_radius_m": 0.75,
            "obstacle_avoidance.scan_freshness_s": 0.3,
            "obstacle_avoidance.telemetry_freshness_s": 1.0,
            "obstacle_avoidance.hysteresis_cost_m": 0.75,
        }
        for name, default in defaults.items():
            if not self._node.has_parameter(name):
                self._node.declare_parameter(name, default)

    def _parameter_float(self, name: str) -> float:
        return float(self._node.get_parameter(name).value)

    def _parameter_string(self, name: str) -> str:
        return str(self._node.get_parameter(name).value)

    def _pose_callback(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        self._pose_received_s = time.monotonic()

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg
        self._fix_received_s = time.monotonic()

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._latest_rel_alt_m = msg.data
        self._altitude_received_s = time.monotonic()

    def _state_callback(self, msg: State) -> None:
        now_s = time.monotonic()
        previous_mode = self._latest_state.mode if self._latest_state is not None else None
        self._latest_state = msg
        self._state_received_s = now_s
        if msg.mode == GUIDED_MODE and previous_mode != GUIDED_MODE:
            self._guided_entry_s = now_s
            self._controller.planner.reset()
        elif msg.mode != GUIDED_MODE and previous_mode == GUIDED_MODE:
            self._controller.planner.reset()

    def _scan_callback(self, msg: LaserScan) -> None:
        received_s = time.monotonic()
        source_stamp_s = float(msg.header.stamp.sec) + msg.header.stamp.nanosec / 1e9
        now_ros_s = self._node.get_clock().now().nanoseconds / 1e9
        converted = prepare_sector_scan(
            ranges_m=msg.ranges,
            angle_min_rad=msg.angle_min,
            angle_increment_rad=msg.angle_increment,
            range_min_m=msg.range_min,
            range_max_m=msg.range_max,
            frame_id=msg.header.frame_id,
            source_stamp_s=source_stamp_s,
            previous_source_stamp_s=self._last_scan_source_stamp_s,
            now_ros_s=now_ros_s,
            received_monotonic_s=received_s,
            config=self._config,
        )
        self._latest_scan = converted
        if (
            math.isfinite(source_stamp_s)
            and source_stamp_s > 0.0
            and (
                self._last_scan_source_stamp_s is None
                or source_stamp_s > self._last_scan_source_stamp_s
            )
        ):
            self._last_scan_source_stamp_s = source_stamp_s

    def initialise(self) -> None:
        try:
            self._waypoint = self.blackboard.get(blackboard_keys.CURRENT_WAYPOINT)
        except KeyError:
            self._waypoint = None
            self._active = False
            return

        now_s = time.monotonic()
        self._active = True
        self._timed_out = False
        self._latest_decision = None
        if self._guided_entry_s is None:
            self._guided_entry_s = now_s
        self._controller.reset()
        self._navigation_clock.reset(now_s)
        self._node.get_logger().info(
            f"{self.name}: obstacle-aware flight to {self._waypoint}"
        )

    def update(self) -> py_trees.common.Status:
        if self._waypoint is None:
            self._node.get_logger().error(f"{self.name}: no waypoint to fly to")
            return py_trees.common.Status.FAILURE
        if self._timed_out:
            self._node.get_logger().error(
                f"{self.name}: waypoint not reached within "
                f"{WAYPOINT_NAV_TIMEOUT_S}s of valid navigation"
            )
            return py_trees.common.Status.FAILURE
        if self._latest_decision is None:
            return py_trees.common.Status.RUNNING
        if self._latest_decision.goal_reached:
            self._node.get_logger().info(
                f"{self.name}: reached waypoint "
                f"({self._latest_decision.goal_distance_m:.2f}m away)"
            )
            return py_trees.common.Status.SUCCESS

        if self._latest_decision.reason is not None:
            self._node.get_logger().warning(
                f"{self.name}: holding: {self._latest_decision.reason}",
                throttle_duration_sec=2.0,
            )
        elif self._latest_decision.goal_distance_m is not None:
            self._node.get_logger().info(
                f"{self.name}: {self._latest_decision.goal_distance_m:.2f}m "
                "to waypoint",
                throttle_duration_sec=2.0,
            )
        return py_trees.common.Status.RUNNING

    def _control_cycle(self) -> None:
        if not self._active or self._waypoint is None:
            return

        now_s = time.monotonic()
        if self._timed_out:
            if self._can_command():
                self._publish_velocity(0.0, 0.0, 0.0)
            if self._latest_decision is not None:
                self._publish_diagnostics(self._latest_decision, now_s)
            return

        if self._latest_state is not None and not self._can_command():
            reason = "DISARMED" if not self._latest_state.armed else "PILOT_CONTROL"
            decision = self._controller.release(reason)
        elif (telemetry := self._telemetry(now_s)) is None:
            decision = self._controller.hold("MISSING_TELEMETRY")
        else:
            goal = self._local_goal(telemetry)
            obstacles = None
            scan_reason = None
            if self._latest_scan is not None and self._latest_pose is not None:
                pose = self._latest_pose.pose
                obstacles = sector_scan_to_snapshot(
                    self._latest_scan.scan,
                    sensor_position=Point2D(pose.position.x, pose.position.y),
                    sensor_heading_rad=telemetry.yaw_enu_rad,
                    obstacle_radius_m=self._config.obstacle_radius_m,
                )
                scan_reason = self._latest_scan.reason
            decision = self._controller.step(
                now_s=now_s,
                goal=goal,
                telemetry=telemetry,
                obstacles=obstacles,
                scan_reason=scan_reason,
            )

        self._latest_decision = decision
        active_elapsed_s = self._navigation_clock.advance(
            now_s, decision.valid_navigation_cycle and not decision.goal_reached
        )
        if active_elapsed_s > WAYPOINT_NAV_TIMEOUT_S:
            self._timed_out = True
            decision = self._controller.hold("NAVIGATION_TIMEOUT")
            self._latest_decision = decision
            self._publish_velocity(0.0, 0.0, 0.0)
        elif decision.should_publish:
            self._publish_velocity(
                decision.east_mps,
                decision.north_mps,
                decision.up_mps,
            )
        self._publish_diagnostics(decision, now_s)

    def _telemetry(self, now_s: float) -> NavigationTelemetry | None:
        if (
            self._latest_pose is None
            or self._latest_fix is None
            or self._latest_rel_alt_m is None
            or self._latest_state is None
            or self._pose_received_s is None
            or self._fix_received_s is None
            or self._altitude_received_s is None
            or self._state_received_s is None
        ):
            return None

        pose = self._latest_pose.pose
        numeric_telemetry = (
            pose.position.x,
            pose.position.y,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            self._latest_rel_alt_m,
        )
        if (
            not self._latest_state.connected
            or self._latest_fix.status.status < NavSatStatus.STATUS_FIX
            or not all(math.isfinite(value) for value in numeric_telemetry)
        ):
            return None

        required_s = self._guided_entry_s
        fresh_after_guided = required_s is not None and all(
            received_s >= required_s
            for received_s in (
                self._pose_received_s,
                self._fix_received_s,
                self._altitude_received_s,
                self._state_received_s,
                self._latest_scan.scan.timestamp_s
                if self._latest_scan is not None
                else -math.inf,
            )
        )
        return NavigationTelemetry(
            east_m=pose.position.x,
            north_m=pose.position.y,
            yaw_enu_rad=self._yaw_from_pose(self._latest_pose),
            latitude=self._latest_fix.latitude,
            longitude=self._latest_fix.longitude,
            relative_altitude_m=self._latest_rel_alt_m,
            armed=self._latest_state.armed,
            mode=self._latest_state.mode,
            pose_received_s=self._pose_received_s,
            fix_received_s=self._fix_received_s,
            altitude_received_s=self._altitude_received_s,
            state_received_s=self._state_received_s,
            fresh_after_guided_entry=fresh_after_guided,
        )

    def _local_goal(self, telemetry: NavigationTelemetry) -> NavigationGoal:
        east_offset_m, north_offset_m = east_north_coordinate_offset_m(
            telemetry.latitude,
            telemetry.longitude,
            self._waypoint.lat,
            self._waypoint.lon,
        )
        return NavigationGoal(
            east_m=telemetry.east_m + east_offset_m,
            north_m=telemetry.north_m + north_offset_m,
            relative_altitude_m=self._waypoint.alt,
        )

    @staticmethod
    def _yaw_from_pose(msg: PoseStamped) -> float:
        orientation = msg.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _publish_velocity(self, east_mps: float, north_mps: float, up_mps: float) -> None:
        message = TwistStamped()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = east_mps
        message.twist.linear.y = north_mps
        message.twist.linear.z = up_mps
        self._velocity_pub.publish(message)

    def _publish_diagnostics(
        self, decision: NavigationDecision, now_s: float
    ) -> None:
        scan_age_s = (
            now_s - self._latest_scan.scan.timestamp_s
            if self._latest_scan is not None
            else math.inf
        )
        status = DiagnosticStatus()
        status.name = "obstacle_aware_waypoint"
        status.hardware_id = "airside"
        status.level = (
            DiagnosticStatus.OK
            if decision.planner_status == "PATH_FOUND"
            else DiagnosticStatus.WARN
        )
        status.message = decision.reason or decision.planner_status
        values = {
            "active": str(self._active).lower(),
            "planner_status": decision.planner_status,
            "reason": decision.reason or "",
            "scan_age_s": f"{scan_age_s:.6f}",
            "pose_age_s": self._format_age(now_s, self._pose_received_s),
            "fix_age_s": self._format_age(now_s, self._fix_received_s),
            "altitude_age_s": self._format_age(now_s, self._altitude_received_s),
            "state_age_s": self._format_age(now_s, self._state_received_s),
            "minimum_clearance_m": self._format_optional(
                decision.minimum_clearance_m
            ),
            "goal_distance_m": self._format_optional(decision.goal_distance_m),
            "path_found_count": str(decision.path_found_count),
            "hold_count": str(decision.hold_count),
            "active_navigation_s": f"{self._navigation_clock.elapsed_s:.6f}",
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_pub.publish(message)

    @staticmethod
    def _format_optional(value: float | None) -> str:
        return "" if value is None else f"{value:.6f}"

    @staticmethod
    def _format_age(now_s: float, received_s: float | None) -> str:
        return "" if received_s is None else f"{now_s - received_s:.6f}"

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._active and self._can_command():
            self._publish_velocity(0.0, 0.0, 0.0)
        self._active = False
        self._controller.planner.reset()

    def _can_command(self) -> bool:
        return (
            self._latest_state is not None
            and self._latest_state.armed
            and self._latest_state.mode == GUIDED_MODE
        )
