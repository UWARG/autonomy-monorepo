#!/usr/bin/env python3
"""End-to-end 3DS mission test against the Docker ArduCopter SITL service.

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
from mavros_msgs.msg import GlobalPositionTarget, HomePosition, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, Float64

EARTH_RADIUS_M = 6_371_000.0


class TestFailure(RuntimeError):
    """Raised when an expected simulator or mission state is not observed."""


class SitlMissionTest(Node):
    """Drive one complete 3DS mission and verify its telemetry."""

    STATE_TOPIC = "mavros/state"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
    REL_ALT_TOPIC = "mavros/global_position/rel_alt"
    HOME_TOPIC = "mavros/home_position/home"
    MAVROS_TARGET_TOPIC = "mavros/setpoint_raw/global"
    MISSION_SETPOINT_TOPIC = "mission/setpoint"
    MISSION_GO_HOME_TOPIC = "mission/go_home"
    SET_MODE_SERVICE = "mavros/set_mode"
    ARMING_SERVICE = "mavros/cmd/arming"

    def __init__(self) -> None:
        super().__init__("sitl_mission_test")

        self._timeout_s = _environment_float("SITL_TEST_TIMEOUT_S", 180.0)
        self._home_lat = _environment_float("SITL_HOME_LAT", -35.363261)
        self._home_lon = _environment_float("SITL_HOME_LON", 149.165230)
        self._home_tolerance_m = _environment_float("SITL_TEST_HOME_TOLERANCE_M", 5.0)
        self._waypoint_offset_east_m = _environment_float(
            "SITL_TEST_WAYPOINT_OFFSET_EAST_M", 10.0
        )
        self._waypoint_alt_m = _environment_float("SITL_TEST_WAYPOINT_ALT_M", 15.0)
        self._takeoff_min_alt_m = _environment_float(
            "SITL_TEST_TAKEOFF_MIN_ALT_M", 14.0
        )
        self._waypoint_radius_m = _environment_float("SITL_TEST_WAYPOINT_RADIUS_M", 1.0)
        self._landed_max_alt_m = _environment_float("SITL_TEST_LANDED_MAX_ALT_M", 1.0)
        self._post_waypoint_wait_s = _environment_float(
            "SITL_TEST_POST_WAYPOINT_WAIT_S", 2.0
        )

        self._state: State | None = None
        self._fix: NavSatFix | None = None
        self._rel_alt_m: float | None = None
        self._home: HomePosition | None = None
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
        self.create_subscription(HomePosition, self.HOME_TOPIC, self._home_callback, 10)
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
            "GUIDED mode state",
            lambda: self._state is not None and self._state.mode == "GUIDED",
        )
        self._arm_vehicle()
        self._wait_for(
            "armed GUIDED state",
            lambda: (
                self._state is not None
                and self._state.armed
                and self._state.mode == "GUIDED"
            ),
        )
        self._wait_for(
            f"airborne altitude of at least {self._takeoff_min_alt_m:.1f}m",
            lambda: (
                self._rel_alt_m is not None
                and self._rel_alt_m >= self._takeoff_min_alt_m
            ),
        )

        waypoint = self._test_waypoint()
        self._wait_for(
            "3DS setpoint subscription",
            lambda: self._setpoint_pub.get_subscription_count() > 0,
        )
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
            f"vehicle within {self._waypoint_radius_m:.1f}m of the waypoint",
            lambda: self._distance_to(waypoint) <= self._waypoint_radius_m,
        )

        # Give the mission tree time to complete its existing photo-capture step
        # before the go-home message preempts its setpoint-serving subtree.
        self._spin_for(self._post_waypoint_wait_s)
        self._wait_for(
            "3DS go-home subscription",
            lambda: self._go_home_pub.get_subscription_count() > 0,
        )
        self.get_logger().info("Publishing go-home command")
        self._go_home_pub.publish(Empty())

        self._wait_for(
            f"landed altitude of at most {self._landed_max_alt_m:.1f}m",
            lambda: (
                self._rel_alt_m is not None
                and self._rel_alt_m <= self._landed_max_alt_m
            ),
        )
        self._wait_for(
            "disarmed state",
            lambda: self._state is not None and not self._state.armed,
        )

    def _set_guided_mode(self) -> None:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "GUIDED"
        response = self._call_service(self._set_mode_client, request, "set GUIDED mode")
        if response is None or not response.mode_sent:
            raise TestFailure(f"MAVROS rejected GUIDED mode request: {response}")

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

    def _assert_known_home_location(self) -> None:
        assert self._fix is not None
        distance_m = _horizontal_distance_m(
            self._fix.latitude, self._fix.longitude, self._home_lat, self._home_lon
        )
        if distance_m > self._home_tolerance_m:
            raise TestFailure(
                "SITL started at an unexpected location: "
                f"{distance_m:.1f}m from the configured home position"
            )
        self.get_logger().info(
            f"PASS: SITL location is within {distance_m:.1f}m of configured home"
        )

    def _test_waypoint(self) -> Coordinate:
        longitude_delta_deg = math.degrees(
            self._waypoint_offset_east_m
            / (EARTH_RADIUS_M * math.cos(math.radians(self._home_lat)))
        )
        return Coordinate(
            lat=self._home_lat,
            lon=self._home_lon + longitude_delta_deg,
            alt=self._waypoint_alt_m,
        )

    def _distance_to(self, target: Coordinate) -> float:
        if not self._has_valid_fix() or self._rel_alt_m is None:
            return math.inf
        assert self._fix is not None
        horizontal_m = _horizontal_distance_m(
            self._fix.latitude, self._fix.longitude, target.lat, target.lon
        )
        return math.sqrt(horizontal_m**2 + (target.alt - self._rel_alt_m) ** 2)

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

    def _state_callback(self, message: State) -> None:
        self._state = message

    def _fix_callback(self, message: NavSatFix) -> None:
        self._fix = message

    def _rel_alt_callback(self, message: Float64) -> None:
        self._rel_alt_m = message.data

    def _home_callback(self, message: HomePosition) -> None:
        self._home = message

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


def _horizontal_distance_m(
    latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float
) -> float:
    """Return a local Earth-surface distance accurate for this short test flight."""

    mean_latitude_rad = math.radians((latitude_a + latitude_b) / 2.0)
    north_m = math.radians(latitude_b - latitude_a) * EARTH_RADIUS_M
    east_m = (
        math.radians(longitude_b - longitude_a)
        * EARTH_RADIUS_M
        * math.cos(mean_latitude_rad)
    )
    return math.hypot(north_m, east_m)


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
