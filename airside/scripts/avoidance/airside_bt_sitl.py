#!/usr/bin/env python3
"""Qualify the airside obstacle-aware behavior against ArduCopter SITL."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from harness_runtime import StableConditionGate
from mavros_msgs.msg import GlobalPositionTarget, ParamEvent, State
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from synthetic_laserscan import (
    crossed_wall_segment,
    point_to_wall_distance_m,
    wall_scan_ranges,
)

BEAM_COUNT = 72
ANGLE_MIN_RAD = -math.pi
ANGLE_INCREMENT_RAD = 2.0 * math.pi / BEAM_COUNT
RANGE_MIN_M = 0.2
RANGE_MAX_M = 50.0
WALL_NORTH_M = 20.0
WALL_HALF_WIDTH_M = 6.0
GOAL_NORTH_M = 40.0
GOAL_ALTITUDE_M = 15.0
GOAL_TOLERANCE_M = 1.0
MIN_CLEARANCE_M = 1.0
EARTH_RADIUS_M = 6_371_000.0
SCAN_PERIOD_S = 0.05
MANAGER_STABLE_S = 2.0


class AirsideBTSitlScenario(Node):
    """Publish synthetic scans and supervise one behavior-tree scenario."""

    def __init__(self, scenario: str) -> None:
        super().__init__("airside_bt_sitl_scenario")
        self.scenario = scenario
        self.lock = threading.RLock()
        self.state: State | None = None
        self.pose: PoseStamped | None = None
        self.fix: NavSatFix | None = None
        self.relative_altitude_m: float | None = None
        self.origin_east_m: float | None = None
        self.origin_north_m: float | None = None
        self.latest_diagnostics: dict[str, str] = {}
        self.diagnostics_count = 0
        self.velocity_count = 0
        self.nonzero_velocity_count = 0
        self.global_setpoint_count = 0
        self.last_velocity = (0.0, 0.0, 0.0)
        self.last_velocity_received_s: float | None = None
        self.parameters: dict[str, dict[str, int | float]] = {}
        self.parameter_expected_count: int | None = None
        self.scan_mode = "normal"
        self.frozen_scan_stamp = None
        self.manager_process: subprocess.Popen[str] | None = None
        self.manager_node_gate = StableConditionGate(MANAGER_STABLE_S)

        self._scenario_subscriptions = [
            self.create_subscription(State, "/mavros/state", self._state_callback, 10),
            self.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                self._pose_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                NavSatFix,
                "/mavros/global_position/global",
                self._fix_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Float64,
                "/mavros/global_position/rel_alt",
                self._altitude_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                DiagnosticArray,
                "/obstacle_avoidance/diagnostics",
                self._diagnostics_callback,
                10,
            ),
            self.create_subscription(
                TwistStamped,
                "/mavros/setpoint_velocity/cmd_vel",
                self._velocity_callback,
                10,
            ),
            self.create_subscription(
                GlobalPositionTarget,
                "/mavros/setpoint_raw/global",
                self._global_setpoint_callback,
                10,
            ),
            self.create_subscription(
                ParamEvent,
                "/mavros/param/event",
                self._parameter_callback,
                qos_profile_sensor_data,
            ),
        ]
        self.scan_publisher = self.create_publisher(
            LaserScan,
            "/obstacle_avoidance/scan",
            qos_profile_sensor_data,
        )
        self.scan_timer = self.create_timer(SCAN_PERIOD_S, self._publish_scan)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.param_pull_client = self.create_client(ParamPull, "/mavros/param/pull")
        self.param_set_client = self.create_client(
            SetParameters,
            "/mavros/param/set_parameters",
        )
        self.param_list_client = self.create_client(
            ListParameters,
            "/mavros/param/list_parameters",
        )
        self.param_get_client = self.create_client(
            GetParameters,
            "/mavros/param/get_parameters",
        )

    def _state_callback(self, message: State) -> None:
        with self.lock:
            self.state = message

    def _pose_callback(self, message: PoseStamped) -> None:
        with self.lock:
            self.pose = message
            if self.origin_east_m is None:
                self.origin_east_m = message.pose.position.x
                self.origin_north_m = message.pose.position.y

    def _fix_callback(self, message: NavSatFix) -> None:
        with self.lock:
            self.fix = message

    def _altitude_callback(self, message: Float64) -> None:
        with self.lock:
            self.relative_altitude_m = message.data

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "obstacle_aware_waypoint":
                continue
            values = {item.key: item.value for item in status.values}
            values["message"] = status.message
            with self.lock:
                self.latest_diagnostics = values
                self.diagnostics_count += 1

    def _velocity_callback(self, message: TwistStamped) -> None:
        velocity = (
            float(message.twist.linear.x),
            float(message.twist.linear.y),
            float(message.twist.linear.z),
        )
        with self.lock:
            self.velocity_count += 1
            self.last_velocity = velocity
            self.last_velocity_received_s = time.monotonic()
            if math.sqrt(sum(component**2 for component in velocity)) > 0.05:
                self.nonzero_velocity_count += 1

    def _global_setpoint_callback(self, _message: GlobalPositionTarget) -> None:
        with self.lock:
            if self.latest_diagnostics.get("active") == "true":
                self.global_setpoint_count += 1

    def _parameter_callback(self, message: ParamEvent) -> None:
        value = message.value
        stored_value: int | float
        if value.type == 2:
            stored_value = int(value.integer_value)
        else:
            stored_value = float(value.double_value)
        with self.lock:
            self.parameter_expected_count = int(message.param_count)
            self.parameters[message.param_id] = {
                "value": stored_value,
                "type": int(value.type),
                "index": int(message.param_index),
            }

    def _relative_position(self) -> tuple[float, float] | None:
        with self.lock:
            if (
                self.pose is None
                or self.origin_east_m is None
                or self.origin_north_m is None
            ):
                return None
            return (
                self.pose.pose.position.x - self.origin_east_m,
                self.pose.pose.position.y - self.origin_north_m,
            )

    def _yaw_enu_rad(self) -> float:
        with self.lock:
            if self.pose is None:
                return 0.0
            orientation = self.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _publish_scan(self) -> None:
        relative_position = self._relative_position()
        if relative_position is None or self.scan_mode == "dropout":
            return
        east_m, north_m = relative_position
        beam_count = BEAM_COUNT
        angle_increment = ANGLE_INCREMENT_RAD
        ranges: tuple[float, ...]
        if self.scan_mode == "partial":
            beam_count = BEAM_COUNT // 2
            ranges = (math.inf,) * beam_count
        elif self.scan_mode == "invalid":
            ranges = (math.nan,) + (math.inf,) * (BEAM_COUNT - 1)
        elif self.scenario == "clear":
            ranges = (math.inf,) * BEAM_COUNT
        else:
            ranges = wall_scan_ranges(
                east_m=east_m,
                north_m=north_m,
                yaw_enu_rad=self._yaw_enu_rad(),
                wall_north_m=WALL_NORTH_M,
                wall_half_width_m=WALL_HALF_WIDTH_M,
                angle_min_rad=ANGLE_MIN_RAD,
                angle_increment_rad=ANGLE_INCREMENT_RAD,
                beam_count=BEAM_COUNT,
                range_max_m=RANGE_MAX_M,
            )

        message = LaserScan()
        if self.scan_mode == "frozen":
            if self.frozen_scan_stamp is None:
                self.frozen_scan_stamp = self.get_clock().now().to_msg()
            message.header.stamp = self.frozen_scan_stamp
        else:
            self.frozen_scan_stamp = None
            message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.angle_min = ANGLE_MIN_RAD
        message.angle_increment = angle_increment
        message.angle_max = ANGLE_MIN_RAD + angle_increment * (beam_count - 1)
        message.range_min = RANGE_MIN_M
        message.range_max = RANGE_MAX_M
        message.scan_time = SCAN_PERIOD_S
        message.time_increment = SCAN_PERIOD_S / beam_count
        message.ranges = list(ranges)
        self.scan_publisher.publish(message)

    def wait_for(self, description: str, predicate: Any, timeout_s: float) -> None:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            if predicate():
                return
            if self.manager_process is not None and self.manager_process.poll() is not None:
                raise RuntimeError(
                    f"engine manager exited with {self.manager_process.returncode} "
                    f"while waiting for {description}"
                )
            time.sleep(0.1)
        raise TimeoutError(f"timed out waiting for {description}")

    def _call_service(self, client: Any, request: Any, timeout_s: float) -> Any:
        if self.manager_process is not None and self.manager_process.poll() is not None:
            raise RuntimeError(
                f"engine manager exited with {self.manager_process.returncode}"
            )
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"service {client.srv_name} unavailable")
        future = client.call_async(request)
        deadline_s = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline_s:
            if (
                self.manager_process is not None
                and self.manager_process.poll() is not None
            ):
                raise RuntimeError(
                    f"engine manager exited with {self.manager_process.returncode}"
                )
            time.sleep(0.05)
        if not future.done():
            raise TimeoutError(f"service {client.srv_name} timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"service {client.srv_name} returned no response")
        return response

    def set_mode(self, mode: str, timeout_s: float = 30.0) -> None:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            request = SetMode.Request()
            request.custom_mode = mode
            response = self._call_service(self.mode_client, request, 5.0)
            if response.mode_sent:
                try:
                    self.wait_for(
                        f"mode {mode}",
                        lambda: self.state is not None and self.state.mode == mode,
                        5.0,
                    )
                    return
                except TimeoutError:
                    pass
            time.sleep(0.5)
        raise TimeoutError(f"could not enter {mode}")

    def arm(self, timeout_s: float = 180.0) -> None:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            request = CommandBool.Request()
            request.value = True
            response = self._call_service(self.arm_client, request, 5.0)
            if response.success:
                self.wait_for(
                    "armed state",
                    lambda: self.state is not None and self.state.armed,
                    10.0,
                )
                return
            time.sleep(1.0)
        raise TimeoutError("flight controller did not arm")

    def download_parameters(self, output_path: Path) -> None:
        request = ParamPull.Request()
        # MAVROS performs the standard PARAM_REQUEST_LIST exchange when the FC
        # connects. Reuse that completed cache; forcing a second full transfer
        # can time out even though the startup exchange was complete.
        request.force_pull = False
        response = self._call_service(self.param_pull_client, request, 60.0)
        if not response.success or response.param_received <= 0:
            raise RuntimeError(f"parameter pull failed: {response}")
        expected = int(response.param_received)
        list_request = ListParameters.Request()
        list_request.depth = ListParameters.Request.DEPTH_RECURSIVE
        list_response = self._call_service(
            self.param_list_client,
            list_request,
            30.0,
        )
        names = sorted(list_response.result.names)
        if len(names) < expected:
            raise RuntimeError(
                "parameter list incomplete; "
                f"listed={len(names)}; expected_at_least={expected}"
            )

        parameters: dict[str, Any] = {}
        for start in range(0, len(names), 200):
            chunk_names = names[start : start + 200]
            get_request = GetParameters.Request()
            get_request.names = chunk_names
            get_response = self._call_service(
                self.param_get_client,
                get_request,
                30.0,
            )
            if len(get_response.values) != len(chunk_names):
                raise RuntimeError(
                    "parameter value response length mismatch; "
                    f"names={len(chunk_names)}; values={len(get_response.values)}"
                )
            for name, value in zip(chunk_names, get_response.values, strict=True):
                parameters[name] = self._parameter_value_to_json(value)

        payload = {
            "expected_count": expected,
            "received_count": len(parameters),
            "parameters": parameters,
        }
        output_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def _parameter_value_to_json(value: ParameterValue) -> dict[str, Any]:
        fields = {
            ParameterType.PARAMETER_BOOL: value.bool_value,
            ParameterType.PARAMETER_INTEGER: value.integer_value,
            ParameterType.PARAMETER_DOUBLE: value.double_value,
            ParameterType.PARAMETER_STRING: value.string_value,
            ParameterType.PARAMETER_BYTE_ARRAY: list(value.byte_array_value),
            ParameterType.PARAMETER_BOOL_ARRAY: list(value.bool_array_value),
            ParameterType.PARAMETER_INTEGER_ARRAY: list(value.integer_array_value),
            ParameterType.PARAMETER_DOUBLE_ARRAY: list(value.double_array_value),
            ParameterType.PARAMETER_STRING_ARRAY: list(value.string_array_value),
        }
        return {
            "type": int(value.type),
            "value": fields.get(value.type),
        }

    def set_integer_parameter(self, name: str, value: int) -> None:
        """Set and verify one integer-valued flight-controller parameter."""

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=value,
                ),
            )
        ]
        response = self._call_service(self.param_set_client, request, 10.0)
        if len(response.results) != 1 or not response.results[0].successful:
            raise RuntimeError(
                f"failed to set {name}={value}; response={response}"
            )

    def write_waypoints(self, output_path: Path) -> None:
        with self.lock:
            if self.fix is None:
                raise RuntimeError("global fix unavailable")
            latitude = float(self.fix.latitude)
            longitude = float(self.fix.longitude)
        goal_latitude = latitude + math.degrees(GOAL_NORTH_M / EARTH_RADIUS_M)
        output_path.write_text(
            "home:\n"
            f"  lat: {latitude:.9f}\n"
            f"  lon: {longitude:.9f}\n"
            f"  alt: {GOAL_ALTITUDE_M:.3f}\n"
            "waypoints:\n"
            f"  - lat: {goal_latitude:.9f}\n"
            f"    lon: {longitude:.9f}\n"
            f"    alt: {GOAL_ALTITUDE_M:.3f}\n",
            encoding="utf-8",
        )

    def start_manager(self, waypoints_path: Path) -> None:
        self.manager_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "engine",
                "manager",
                "--ros-args",
                "-p",
                f"waypoints_file:={waypoints_path}",
            ],
            text=True,
            start_new_session=True,
        )

    def manager_ready(self) -> bool:
        """Return true after the engine ROS node has been stable long enough."""

        manager_present = ("engine_manager", "/") in set(
            self.get_node_names_and_namespaces()
        )
        return self.manager_node_gate.observe(manager_present)

    def require_manager_running(self) -> None:
        """Fail the scenario if the behavior-tree process exited unexpectedly."""

        if self.manager_process is None:
            raise RuntimeError("engine manager was not started")
        return_code = self.manager_process.poll()
        if return_code is not None:
            raise RuntimeError(f"engine manager exited unexpectedly with {return_code}")

    def stop_manager(self) -> None:
        if self.manager_process is None or self.manager_process.poll() is not None:
            return
        os.killpg(self.manager_process.pid, signal.SIGINT)
        try:
            self.manager_process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.manager_process.pid, signal.SIGTERM)
            try:
                self.manager_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.manager_process.pid, signal.SIGKILL)
            self.manager_process.wait(timeout=5.0)

    def readiness_complete(self) -> bool:
        with self.lock:
            return bool(
                self.state is not None
                and self.state.connected
                and self.pose is not None
                and self.fix is not None
                and self.fix.status.status >= NavSatStatus.STATUS_FIX
                and math.isfinite(self.fix.latitude)
                and math.isfinite(self.fix.longitude)
                and self.relative_altitude_m is not None
            )

    def readiness_details(self) -> dict[str, Any]:
        """Return a serializable view of each MAVROS readiness input."""

        with self.lock:
            details = {
                "state_received": self.state is not None,
                "connected": bool(self.state and self.state.connected),
                "pose_received": self.pose is not None,
                "fix_received": self.fix is not None,
                "fix_status": self.fix.status.status if self.fix is not None else None,
                "altitude_received": self.relative_altitude_m is not None,
            }
        details["nodes"] = self.get_node_names_and_namespaces()
        details["topics"] = sorted(name for name, _types in self.get_topic_names_and_types())
        return details

    def active_navigation(self) -> bool:
        with self.lock:
            return self.latest_diagnostics.get("active") == "true"

    def goal_reached(self) -> bool:
        relative_position = self._relative_position()
        with self.lock:
            altitude_m = self.relative_altitude_m
        if relative_position is None or altitude_m is None:
            return False
        east_m, north_m = relative_position
        return (
            math.sqrt(
                east_m**2
                + (north_m - GOAL_NORTH_M) ** 2
                + (altitude_m - GOAL_ALTITUDE_M) ** 2
            )
            <= GOAL_TOLERANCE_M
        )

    def snapshot(self) -> dict[str, Any]:
        relative_position = self._relative_position()
        with self.lock:
            state = self.state
            altitude_m = self.relative_altitude_m
            diagnostics = dict(self.latest_diagnostics)
            return {
                "utc": datetime.now(timezone.utc).isoformat(),
                "east_m": relative_position[0] if relative_position else None,
                "north_m": relative_position[1] if relative_position else None,
                "relative_altitude_m": altitude_m,
                "armed": state.armed if state is not None else False,
                "mode": state.mode if state is not None else None,
                "velocity": self.last_velocity,
                "velocity_count": self.velocity_count,
                "global_setpoint_count": self.global_setpoint_count,
                "diagnostics": diagnostics,
            }


def evaluate_summary(summary: dict[str, Any]) -> None:
    scenario = str(summary["scenario"])
    common = (
        summary["global_setpoint_count"] == 0
        and summary["diagnostics_count"] > 0
        and summary["failure_reason"] is None
    )
    if scenario in {"invalid", "partial"}:
        passed = (
            common
            and summary["nonzero_velocity_count"] == 0
            and summary["planner_status"] == "NO_PATH"
            and summary["goal_reached_at_s"] is None
        )
    else:
        passed = (
            common
            and summary["goal_reached_at_s"] is not None
            and summary["goal_reached_at_s"] <= 90.0
            and not summary["breached"]
            and summary["planner_path_found_count"] > 0
            and summary["planner_status"] == "PATH_FOUND"
            and summary["zero_at_goal"]
        )
        if scenario != "clear":
            passed = passed and summary["min_wall_dist_m"] >= MIN_CLEARANCE_M
        if scenario == "wall":
            passed = passed and summary["planner_hold_count"] == 0
        if scenario in {"dropout", "frozen", "pilot_takeover"}:
            passed = passed and summary["stop_observed"]
    summary["verdict"] = "PASS" if passed else "FAIL"


def run_scenario(node: AirsideBTSitlScenario, args: argparse.Namespace) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "scenario": args.scenario,
        "verdict": "FAIL",
        "failure_stage": None,
        "failure_reason": None,
        "armed": False,
        "goal_reached_at_s": None,
        "min_wall_dist_m": None,
        "breached": False,
        "planner_status": "NOT_STARTED",
        "planner_reason": None,
        "planner_path_found_count": 0,
        "planner_hold_count": 0,
        "diagnostics_count": 0,
        "velocity_count": 0,
        "nonzero_velocity_count": 0,
        "global_setpoint_count": 0,
        "stop_observed": False,
        "zero_at_goal": False,
    }
    stage = "mavros_readiness"
    log_path = Path(args.log_jsonl)
    try:
        if args.scenario in {"invalid", "partial"}:
            node.scan_mode = args.scenario
        node.wait_for(
            "fresh MAVROS telemetry",
            node.readiness_complete,
            args.readiness_timeout,
        )
        stage = "flight_controller_configuration"
        node.set_integer_parameter("AVOID_ENABLE", 0)
        stage = "parameter_download"
        node.download_parameters(Path(args.params_json))
        stage = "waypoint_generation"
        waypoints_path = Path(args.waypoints_file)
        node.write_waypoints(waypoints_path)
        stage = "manager_startup"
        node.start_manager(waypoints_path)
        stage = "manager_readiness"
        node.wait_for("stable engine manager ROS node", node.manager_ready, 60.0)
        stage = "guided_mode"
        node.set_mode("GUIDED")
        stage = "arming"
        node.arm()
        stage = "behavior_activation"
        node.wait_for("obstacle-aware lapping behavior", node.active_navigation, 180.0)

        navigation_start_s = time.monotonic()
        fault_started_s: float | None = None
        fault_finished = False
        previous_position = node._relative_position()
        min_wall_distance_m = math.inf
        pilot_quiet_start_count: int | None = None
        pilot_quiet_deadline_s: float | None = None

        stage = "scenario_monitor"
        with log_path.open("w", encoding="utf-8") as log:
            while time.monotonic() - navigation_start_s <= args.duration:
                node.require_manager_running()
                now_s = time.monotonic()
                elapsed_s = now_s - navigation_start_s
                snapshot = node.snapshot()
                snapshot["t"] = round(elapsed_s, 3)
                log.write(json.dumps(snapshot, sort_keys=True) + "\n")
                log.flush()

                position = node._relative_position()
                if position is not None and args.scenario != "clear":
                    east_m, north_m = position
                    wall_distance_m = point_to_wall_distance_m(
                        east_m,
                        north_m,
                        wall_north_m=WALL_NORTH_M,
                        wall_half_width_m=WALL_HALF_WIDTH_M,
                    )
                    min_wall_distance_m = min(min_wall_distance_m, wall_distance_m)
                    if previous_position is not None and crossed_wall_segment(
                        previous_position,
                        position,
                        wall_north_m=WALL_NORTH_M,
                        wall_half_width_m=WALL_HALF_WIDTH_M,
                    ):
                        summary["breached"] = True
                    previous_position = position

                    if (
                        not fault_finished
                        and fault_started_s is None
                        and north_m >= 5.0
                        and args.scenario in {"dropout", "frozen"}
                    ):
                        fault_started_s = now_s
                        node.scan_mode = args.scenario
                    if (
                        fault_started_s is not None
                        and not fault_finished
                        and now_s - fault_started_s >= 1.2
                        and args.scenario in {"dropout", "frozen"}
                    ):
                        with node.lock:
                            speed = math.sqrt(
                                sum(component**2 for component in node.last_velocity)
                            )
                        summary["stop_observed"] = speed <= 0.05 and north_m < WALL_NORTH_M
                        node.scan_mode = "normal"
                        fault_finished = True

                    if (
                        not fault_finished
                        and fault_started_s is None
                        and north_m >= 5.0
                        and args.scenario == "pilot_takeover"
                    ):
                        node.set_mode("LOITER")
                        fault_started_s = time.monotonic()
                        time.sleep(0.3)
                        with node.lock:
                            pilot_quiet_start_count = node.velocity_count
                        pilot_quiet_deadline_s = time.monotonic() + 1.5
                    if (
                        pilot_quiet_deadline_s is not None
                        and not fault_finished
                        and now_s >= pilot_quiet_deadline_s
                    ):
                        with node.lock:
                            summary["stop_observed"] = (
                                node.velocity_count == pilot_quiet_start_count
                            )
                        node.set_mode("GUIDED")
                        fault_finished = True

                if args.scenario in {"invalid", "partial"}:
                    node.scan_mode = args.scenario
                    if elapsed_s >= 5.0:
                        break
                elif node.goal_reached():
                    summary["goal_reached_at_s"] = round(elapsed_s, 3)
                    time.sleep(0.3)
                    with node.lock:
                        summary["zero_at_goal"] = (
                            math.sqrt(
                                sum(component**2 for component in node.last_velocity)
                            )
                            <= 0.05
                        )
                    break
                time.sleep(0.1)

        node.require_manager_running()
        with node.lock:
            diagnostics = dict(node.latest_diagnostics)
            summary.update(
                {
                    "armed": bool(node.state and node.state.armed),
                    "diagnostics_count": node.diagnostics_count,
                    "velocity_count": node.velocity_count,
                    "nonzero_velocity_count": node.nonzero_velocity_count,
                    "global_setpoint_count": node.global_setpoint_count,
                    "planner_status": diagnostics.get("planner_status", "NOT_STARTED"),
                    "planner_reason": diagnostics.get("reason") or None,
                    "planner_path_found_count": int(
                        diagnostics.get("path_found_count", "0")
                    ),
                    "planner_hold_count": int(diagnostics.get("hold_count", "0")),
                }
            )
        summary["min_wall_dist_m"] = (
            round(min_wall_distance_m, 3)
            if min_wall_distance_m < math.inf
            else None
        )
        evaluate_summary(summary)
    except Exception as error:  # noqa: BLE001 - formal scenario boundary
        summary["failure_stage"] = stage
        summary["failure_reason"] = f"{type(error).__name__}: {error}"
        summary["readiness"] = node.readiness_details()
    finally:
        node.stop_manager()
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=[
            "clear",
            "wall",
            "dropout",
            "frozen",
            "invalid",
            "partial",
            "pilot_takeover",
        ],
        required=True,
    )
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--params-json", required=True)
    parser.add_argument("--log-jsonl", required=True)
    parser.add_argument("--waypoints-file", required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--readiness-timeout", type=float, default=180.0)
    args = parser.parse_args()

    for path_text in (
        args.summary_json,
        args.params_json,
        args.log_jsonl,
        args.waypoints_file,
    ):
        Path(path_text).parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = AirsideBTSitlScenario(args.scenario)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    spin_thread.start()
    try:
        summary = run_scenario(node, args)
    finally:
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=5.0)

    Path(args.summary_json).write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
