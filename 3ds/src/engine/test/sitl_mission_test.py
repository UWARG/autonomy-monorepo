#!/usr/bin/env python3
"""
End-to-end 3DS mission test against the Docker ArduCopter SITL service.

The node acts as an automated pilot and referee. It does not forward commands
between 3DS and the flight controller: MAVROS keeps that production command
and telemetry path intact.
"""

from __future__ import annotations

import math
import os
import sys
import time
from collections.abc import Callable

import rclpy
from airside_interfaces.msg import Coordinate
from engine.constants import (
    GUIDED_MODE,
    RETURN_ALTITUDE_M,
    TAKEOFF_ALTITUDE_M,
    TAKEOFF_ALTITUDE_TOLERANCE_M,
    WAYPOINT_ACCEPTANCE_RADIUS_M,
)
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, Float64
from utils.src.constants import EARTH_RADIUS_M
from utils.src.waypoint_utils import east_north_coordinate_offset_m


class TestFailure(RuntimeError):
    """Raised when an expected simulator or mission state is not observed."""


class SitlMissionTest(Node):
    """Drive one complete 3DS mission and verify its telemetry."""

    STATE_TOPIC = "mavros/state"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
    REL_ALT_TOPIC = "mavros/global_position/rel_alt"
    MAVROS_TARGET_TOPIC = "mavros/setpoint_raw/global"
    MISSION_SETPOINT_TOPIC = "mission/setpoint"
    MISSION_GO_HOME_TOPIC = "mission/go_home"
    SET_MODE_SERVICE = "mavros/set_mode"
    ARMING_SERVICE = "mavros/cmd/arming"

    DEFAULT_HOME_LAT = -35.363261
    DEFAULT_HOME_LON = 149.165230
    DEFAULT_TIMEOUT_S = 180.0

    HOME_TOLERANCE_M = 5.0
    WAYPOINT_OFFSET_EAST_M = 10.0
    WAYPOINT_ALT_M = TAKEOFF_ALTITUDE_M
    TAKEOFF_MIN_ALT_M = TAKEOFF_ALTITUDE_M - TAKEOFF_ALTITUDE_TOLERANCE_M
    WAYPOINT_RADIUS_M = WAYPOINT_ACCEPTANCE_RADIUS_M
    LANDED_MAX_ALT_M = 1.0
    POST_WAYPOINT_WAIT_S = 1.5

    def __init__(self) -> None:
        super().__init__("sitl_mission_test")

        self._timeout_s = _environment_float(
            "SITL_TEST_TIMEOUT_S", self.DEFAULT_TIMEOUT_S
        )
        self._home = Coordinate(
            lat=_environment_float("SITL_HOME_LAT", self.DEFAULT_HOME_LAT),
            lon=_environment_float("SITL_HOME_LON", self.DEFAULT_HOME_LON),
            alt=RETURN_ALTITUDE_M,
        )

        self._state: State | None = None
        self._fix: NavSatFix | None = None
        self._rel_alt_m: float | None = None
        self._forwarded_target: GlobalPositionTarget | None = None

        self.create_subscription(State, self.STATE_TOPIC, self._state_callback, 10)
        self.create_subscription(
            NavSatFix,
            self.GLOBAL_POSITION_TOPIC,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            self.REL_ALT_TOPIC,
            self._rel_alt_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GlobalPositionTarget,
            self.MAVROS_TARGET_TOPIC,
            self._target_callback,
            10,
        )

        self._setpoint_pub = self.create_publisher(
            Coordinate, self.MISSION_SETPOINT_TOPIC, 10
        )
        self._go_home_pub = self.create_publisher(Empty, self.MISSION_GO_HOME_TOPIC, 10)
        self._set_mode_client = self.create_client(SetMode, self.SET_MODE_SERVICE)
        self._arming_client = self.create_client(CommandBool, self.ARMING_SERVICE)

    def run(self) -> None:
        """Run the mission and raise TestFailure if a phase does not complete."""
        self._wait_for(
            "MAVROS connection",
            lambda: self._state is not None and self._state.connected,
        )
        self._wait_for("initial GPS fix", self._has_valid_fix)
        self._assert_known_home_location()
        self._wait_for(
            "3DS engine setpoint subscription",
            lambda: self._setpoint_pub.get_subscription_count() > 0,
        )

        self._set_guided_mode()
        self._wait_for(
            f"{GUIDED_MODE} mode state",
            lambda: self._state is not None and self._state.mode == GUIDED_MODE,
        )
        self._arm_vehicle()
        self._wait_for(
            f"armed {GUIDED_MODE} state",
            lambda: (
                self._state is not None
                and self._state.armed
                and self._state.mode == GUIDED_MODE
            ),
        )
        self._wait_for(
            f"airborne altitude of at least {self.TAKEOFF_MIN_ALT_M:.1f}m",
            lambda: (
                self._rel_alt_m is not None
                and self._rel_alt_m >= self.TAKEOFF_MIN_ALT_M
            ),
        )

        waypoint = self._test_waypoint()
        self.get_logger().info(
            "Publishing test waypoint "
            f"lat={waypoint.lat:.7f}, lon={waypoint.lon:.7f}, alt={waypoint.alt:.1f}m"
        )
        self._setpoint_pub.publish(waypoint)
        self._wait_for(
            "3DS forwarding the waypoint through MAVROS",
            lambda: self._target_matches(waypoint),
        )
        self._wait_for(
            f"vehicle within {self.WAYPOINT_RADIUS_M:.1f}m of the waypoint",
            lambda: self._distance_to(waypoint) <= self.WAYPOINT_RADIUS_M,
        )

        # Let the 500ms behavior tree tick and complete FlyToSetpoint arrival
        # before the go-home message preempts the setpoint-serving subtree.
        self._spin_for(self.POST_WAYPOINT_WAIT_S)

        self._wait_for(
            "3DS go-home subscription",
            lambda: self._go_home_pub.get_subscription_count() > 0,
        )
        self.get_logger().info("Publishing go-home command")
        self._go_home_pub.publish(Empty())

        self._wait_for(
            "3DS forwarding the launch point through MAVROS",
            self._target_matches_home,
        )
        self._wait_for(
            f"landed altitude of at most {self.LANDED_MAX_ALT_M:.1f}m",
            lambda: (
                self._rel_alt_m is not None
                and self._rel_alt_m <= self.LANDED_MAX_ALT_M
            ),
        )
        self._wait_for(
            "disarmed state",
            lambda: self._state is not None and not self._state.armed,
        )
        self._assert_known_home_location(tolerance_m=self.WAYPOINT_RADIUS_M)

    def _set_guided_mode(self) -> None:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = GUIDED_MODE
        response = self._call_service(
            self._set_mode_client, request, f"set {GUIDED_MODE} mode"
        )
        if response is None or not response.mode_sent:
            raise TestFailure(
                f"MAVROS rejected {GUIDED_MODE} mode request: {response}"
            )

    def _arm_vehicle(self) -> None:
        deadline = time.monotonic() + self._timeout_s
        last_response = None
        request = CommandBool.Request()
        request.value = True

        while time.monotonic() < deadline:
            response = self._call_service(self._arming_client, request, "arm vehicle")
            if response is not None and response.success:
                return

            last_response = response
            self.get_logger().warning(
                "ArduCopter rejected arming while it completes pre-arm checks; retrying"
            )
            self._spin_for(1.0)

        raise TestFailure(
            "MAVROS did not accept an arming request before the timeout: "
            f"{last_response}"
        )

    def _call_service(self, client, request, action: str):
        self._wait_for(f"{action} service", client.service_is_ready)
        self.get_logger().info(f"Requesting: {action}")
        future = client.call_async(request)
        self._wait_for(f"{action} response", future.done)
        try:
            return future.result()
        except Exception as error:
            raise TestFailure(f"{action} service failed: {error}") from error

    def _wait_for(self, description: str, condition: Callable[[], bool]) -> None:
        deadline = time.monotonic() + self._timeout_s
        self.get_logger().info(f"Waiting for {description}")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                self.get_logger().info(f"PASS: {description}")
                return
        raise TestFailure(
            f"Timed out after {self._timeout_s:.0f}s waiting for {description}"
        )

    def _spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _has_valid_fix(self) -> bool:
        return (
            self._fix is not None
            and math.isfinite(self._fix.latitude)
            and math.isfinite(self._fix.longitude)
        )

    def _current_position(self) -> Coordinate:
        assert self._fix is not None
        return Coordinate(
            lat=self._fix.latitude,
            lon=self._fix.longitude,
            alt=self._rel_alt_m if self._rel_alt_m is not None else math.nan,
        )

    def _assert_known_home_location(self, tolerance_m: float | None = None) -> None:
        max_distance_m = (
            self.HOME_TOLERANCE_M if tolerance_m is None else tolerance_m
        )
        distance_m = _horizontal_distance_m(self._current_position(), self._home)
        if distance_m > max_distance_m:
            raise TestFailure(
                "SITL vehicle is at an unexpected location: "
                f"{distance_m:.1f}m from the configured home position "
                f"(tolerance {max_distance_m:.1f}m)"
            )
        self.get_logger().info(
            f"PASS: vehicle location is within {distance_m:.1f}m of configured home"
        )

    def _test_waypoint(self) -> Coordinate:
        longitude_delta_deg = math.degrees(
            self.WAYPOINT_OFFSET_EAST_M
            / (EARTH_RADIUS_M * math.cos(math.radians(self._home.lat)))
        )
        return Coordinate(
            lat=self._home.lat,
            lon=self._home.lon + longitude_delta_deg,
            alt=self.WAYPOINT_ALT_M,
        )

    def _distance_to(self, target: Coordinate) -> float:
        if not self._has_valid_fix() or self._rel_alt_m is None:
            return math.inf
        current = self._current_position()
        horizontal_m = _horizontal_distance_m(current, target)
        return math.hypot(horizontal_m, target.alt - current.alt)

    def _target_matches(self, target: Coordinate) -> bool:
        message = self._forwarded_target
        if message is None:
            return False
        return (
            message.coordinate_frame == GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            and math.isclose(message.latitude, target.lat, abs_tol=1e-7)
            and math.isclose(message.longitude, target.lon, abs_tol=1e-7)
            and math.isclose(message.altitude, target.alt, abs_tol=0.01)
        )

    def _target_matches_home(self) -> bool:
        message = self._forwarded_target
        if message is None:
            return False
        forwarded_point = Coordinate(
            lat=message.latitude,
            lon=message.longitude,
            alt=message.altitude,
        )
        return (
            message.coordinate_frame == GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            and math.isclose(message.altitude, self._home.alt, abs_tol=0.01)
            and _horizontal_distance_m(forwarded_point, self._home)
            <= self.HOME_TOLERANCE_M
        )

    def _state_callback(self, message: State) -> None:
        self._state = message

    def _fix_callback(self, message: NavSatFix) -> None:
        self._fix = message

    def _rel_alt_callback(self, message: Float64) -> None:
        self._rel_alt_m = message.data

    def _target_callback(self, message: GlobalPositionTarget) -> None:
        self._forwarded_target = message


def _environment_float(name: str, default: float) -> float:
    raw_value = os.environ.get(name, str(default))
    try:
        value = float(raw_value)
    except ValueError as error:
        raise TestFailure(f"{name} must be a number, got {raw_value!r}") from error
    if not math.isfinite(value):
        raise TestFailure(f"{name} must be finite, got {raw_value!r}")
    return value


def _horizontal_distance_m(point_a: Coordinate, point_b: Coordinate) -> float:
    """Return a local Earth-surface distance accurate for this short test flight."""
    east_m, north_m = east_north_coordinate_offset_m(
        point_a.lat, point_a.lon, point_b.lat, point_b.lon
    )
    return math.hypot(east_m, north_m)


def main() -> int:
    rclpy.init()
    node = SitlMissionTest()
    try:
        node.run()
        node.get_logger().info("PASS: 3DS SITL mission integration test completed")
        return 0
    except TestFailure as error:
        node.get_logger().error(f"FAIL: {error}")
        return 1
    except Exception as error:  # noqa: BLE001 - test process exit boundary
        node.get_logger().error(f"FAIL: unexpected test error: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
