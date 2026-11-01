#!/usr/bin/env python3
"""Run PR #181's live-backup obstacle path against ArduCopter SITL."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from airside_interfaces.msg import Coordinate, Obstacle, ObstacleSnapshotStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

EARTH_RADIUS_M = 6_371_000.0
ALTITUDE_M = 15.0
GOAL_NORTH_M = 40.0
OBSTACLE_NORTH_M = 20.0
SCENARIOS = ("clear", "detour", "dropout", "no_path", "pilot_takeover")


def offset(lat: float, lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    new_lat = lat + math.degrees(north_m / EARTH_RADIUS_M)
    new_lon = lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return new_lat, new_lon


class Scenario(Node):
    def __init__(self, name: str) -> None:
        super().__init__("aeac_backup_sitl_readiness")
        self.scenario = name
        self.state: State | None = None
        self.fix: NavSatFix | None = None
        self.altitude_m: float | None = None
        self.pose: PoseStamped | None = None
        self.home_pose: tuple[float, float] | None = None
        self.home_wgs84: tuple[float, float] | None = None
        self.send_target = False
        self.send_traffic = False
        self.sequence = time.time_ns()
        self.setpoint_count = 0
        self.last_setpoint: GlobalPositionTarget | None = None
        self.diagnostic = ""
        self.diagnostics: set[str] = set()
        self.min_clearance_m = math.inf
        self.max_speed_mps = 0.0
        self.last_track: tuple[float, float, float] | None = None

        self.create_subscription(State, "/mavros/state", self._state, 10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._fix, qos_profile_sensor_data)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._alt, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose, qos_profile_sensor_data)
        self.create_subscription(GlobalPositionTarget, "/mavros/setpoint_raw/global", self._setpoint, 10)
        self.create_subscription(DiagnosticArray, "/position_controller/diagnostics", self._diagnostics, 10)
        self.target_pub = self.create_publisher(Coordinate, "/position_controller/target", 10)
        self.obstacle_pub = self.create_publisher(Obstacle, "/position_controller/obstacle", 10)
        self.status_pub = self.create_publisher(ObstacleSnapshotStatus, "/position_controller/obstacle_snapshot", 10)
        self.create_timer(0.2, self.publish_target)
        self.create_timer(1.0, self.publish_traffic)
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.param_client = self.create_client(GetParameters, "/mavros/param/get_parameters")

    def _state(self, msg: State) -> None:
        self.state = msg

    def _fix(self, msg: NavSatFix) -> None:
        self.fix = msg

    def _alt(self, msg: Float64) -> None:
        self.altitude_m = msg.data

    def _pose(self, msg: PoseStamped) -> None:
        self.pose = msg
        point = (msg.pose.position.x, msg.pose.position.y)
        if self.home_pose is None:
            self.home_pose = point
        if not self.send_target:
            return
        east_m, north_m = self.position()
        now_s = time.monotonic()
        if self.last_track is not None:
            old_east, old_north, old_s = self.last_track
            elapsed_s = now_s - old_s
            if elapsed_s >= 0.08:
                speed = math.hypot(east_m - old_east, north_m - old_north) / elapsed_s
                self.max_speed_mps = max(self.max_speed_mps, speed)
        self.last_track = (east_m, north_m, now_s)
        if self.scenario == "detour":
            self.min_clearance_m = min(
                self.min_clearance_m,
                math.hypot(east_m, north_m - OBSTACLE_NORTH_M) - 5.0,
            )

    def _setpoint(self, msg: GlobalPositionTarget) -> None:
        self.setpoint_count += 1
        self.last_setpoint = msg

    def _diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == "position_controller/live_traffic":
                self.diagnostic = status.message
                self.diagnostics.add(status.message)

    def position(self) -> tuple[float, float]:
        if self.pose is None or self.home_pose is None:
            return 0.0, 0.0
        return (
            self.pose.pose.position.x - self.home_pose[0],
            self.pose.pose.position.y - self.home_pose[1],
        )

    def publish_target(self) -> None:
        if not self.send_target or self.home_wgs84 is None:
            return
        distance_m = 150.0 if self.scenario == "no_path" else GOAL_NORTH_M
        lat, lon = offset(*self.home_wgs84, 0.0, distance_m)
        self.target_pub.publish(Coordinate(lat=lat, lon=lon, alt=ALTITUDE_M))

    def aircraft(self) -> list[tuple[int, float, float, float]]:
        if self.scenario == "detour":
            return [(2, 0.0, OBSTACLE_NORTH_M, 5.0)]
        if self.scenario == "no_path":
            return [
                (2 + i, 80.0 * math.sin(math.radians(i * 30)),
                 80.0 * math.cos(math.radians(i * 30)), 20.0)
                for i in range(12)
            ]
        return []

    def publish_traffic(self) -> None:
        if not self.send_traffic or self.home_wgs84 is None:
            return
        self.sequence += 1
        stamp = self.get_clock().now().to_msg()
        indices = []
        for index, east_m, north_m, keepaway_m in self.aircraft():
            lat, lon = offset(*self.home_wgs84, east_m, north_m)
            obstacle = Obstacle()
            obstacle.header.stamp = stamp
            obstacle.header.frame_id = "wgs84"
            obstacle.sequence = self.sequence
            obstacle.aircraft_index = index
            obstacle.horizontal_keep_away = keepaway_m
            obstacle.vertical_keep_away = 5.0
            obstacle.position = Coordinate(lat=lat, lon=lon, alt=ALTITUDE_M)
            self.obstacle_pub.publish(obstacle)
            indices.append(index)
        status = ObstacleSnapshotStatus()
        status.header.stamp = stamp
        status.sequence = self.sequence
        status.connected = True
        status.healthy = True
        status.aircraft_indices = indices
        self.status_pub.publish(status)

    def wait_for(self, label: str, predicate, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError(f"timed out waiting for {label}")

    def pump(self, duration_s: float) -> None:
        end_s = time.monotonic() + duration_s
        while time.monotonic() < end_s:
            rclpy.spin_once(self, timeout_sec=0.1)

    def call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"service {client.srv_name} unavailable")
        future = client.call_async(request)
        self.wait_for(client.srv_name, future.done, timeout_s)
        response = future.result()
        if response is None:
            raise RuntimeError(f"service {client.srv_name} returned no response")
        return response

    def set_mode(self, mode: str) -> None:
        for _ in range(10):
            response = self.call(self.mode_client, SetMode.Request(custom_mode=mode))
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
        raise RuntimeError(f"FCU would not enter {mode}")

    def arm(self) -> None:
        for _ in range(60):
            response = self.call(self.arm_client, CommandBool.Request(value=True))
            if response.success:
                self.wait_for("armed state", lambda: self.state is not None and self.state.armed, 10.0)
                return
            self.pump(1.0)
        raise RuntimeError("FCU did not arm")

    def check_speed_parameter(self) -> float:
        response = self.call(self.param_client, GetParameters.Request(names=["WPNAV_SPEED"]))
        if len(response.values) != 1:
            raise RuntimeError("WPNAV_SPEED missing from the SITL FCU")
        value = response.values[0]
        if value.type == ParameterType.PARAMETER_INTEGER:
            raw_value = float(value.integer_value)
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            raw_value = value.double_value
        else:
            raise RuntimeError("WPNAV_SPEED has an unsupported parameter type")
        speed_mps = raw_value / 100.0
        if speed_mps > 1.01:
            raise RuntimeError(f"WPNAV_SPEED exceeds 1 m/s: {speed_mps:.2f}")
        return speed_mps

    def check_setpoint_owner(self) -> None:
        publishers = self.get_publishers_info_by_topic("/mavros/setpoint_raw/global")
        owners = [(item.node_name, item.node_namespace) for item in publishers]
        if owners != [("position_controller", "/")]:
            raise RuntimeError(f"unexpected MAVROS setpoint publishers: {owners}")

    def run(self) -> dict[str, object]:
        self.wait_for(
            "connected FCU, GPS fix, and local pose",
            lambda: self.state is not None
            and self.state.connected
            and self.fix is not None
            and self.fix.status.status >= NavSatStatus.STATUS_FIX
            and self.pose is not None,
            120.0,
        )
        self.wait_for(
            "position controller publisher",
            lambda: bool(self.get_publishers_info_by_topic("/mavros/setpoint_raw/global")),
            15.0,
        )
        self.check_setpoint_owner()
        configured_speed_mps = self.check_speed_parameter()
        assert self.fix is not None
        self.home_wgs84 = (self.fix.latitude, self.fix.longitude)
        self.set_mode("GUIDED")
        self.arm()
        response = self.call(self.takeoff_client, CommandTOL.Request(altitude=ALTITUDE_M))
        if not response.success:
            raise RuntimeError(f"takeoff rejected with result {response.result}")
        self.wait_for(
            "takeoff altitude",
            lambda: self.altitude_m is not None and self.altitude_m >= ALTITUDE_M - 2.0,
            70.0,
        )
        self.send_traffic = True
        self.publish_traffic()
        self.send_target = True
        self.publish_target()

        if self.scenario == "no_path":
            self.wait_for("no-route hold", lambda: self.diagnostic == "hold", 10.0)
            self.pump(2.0)
            if math.hypot(*self.position()) > 2.0:
                raise RuntimeError("vehicle moved toward a blocked route")
        elif self.scenario == "dropout":
            self.wait_for("initial movement", lambda: self.position()[1] >= 5.0, 35.0)
            traffic_loss_position = self.position()
            self.send_traffic = False
            self.wait_for("stale traffic hold", lambda: self.diagnostic == "STALE_TRAFFIC", 7.0)
            self.pump(2.0)
            if math.dist(traffic_loss_position, self.position()) > 4.0:
                raise RuntimeError("vehicle progressed too far after traffic loss")
            self.send_traffic = True
            self.publish_traffic()
            self.wait_for("resumed goal", lambda: self.position()[1] >= 38.0, 90.0)
        elif self.scenario == "pilot_takeover":
            self.wait_for("initial movement", lambda: self.position()[1] >= 5.0, 35.0)
            self.set_mode("LOITER")
            self.send_target = False
            assert self.home_wgs84 is not None
            lat, lon = offset(*self.home_wgs84, 0.0, GOAL_NORTH_M)
            self.target_pub.publish(Coordinate(lat=lat, lon=lon, alt=ALTITUDE_M))
            self.pump(0.3)
            quiet_count = self.setpoint_count
            self.pump(1.5)
            if self.setpoint_count != quiet_count:
                raise RuntimeError("controller sent a setpoint after pilot takeover")
            self.set_mode("GUIDED")
            self.pump(0.8)
            if self.setpoint_count != quiet_count:
                raise RuntimeError("LOITER target replayed on GUIDED resume")
            self.send_target = True
            self.publish_target()
            self.wait_for("resumed goal", lambda: self.position()[1] >= 38.0, 90.0)
        else:
            self.wait_for("goal", lambda: self.position()[1] >= 38.0, 110.0)

        if self.scenario == "detour" and self.min_clearance_m < 0.5:
            raise RuntimeError(f"keep-away breached: clearance {self.min_clearance_m:.2f} m")
        if self.max_speed_mps > 1.0:
            raise RuntimeError(f"observed speed exceeded 1 m/s: {self.max_speed_mps:.2f} m/s")
        return {
            "scenario": self.scenario,
            "result": "PASS",
            "configured_speed_mps": configured_speed_mps,
            "max_observed_speed_mps": round(self.max_speed_mps, 3),
            "min_clearance_beyond_keepaway_m": (
                None if math.isinf(self.min_clearance_m) else round(self.min_clearance_m, 3)
            ),
            "setpoint_count": self.setpoint_count,
            "diagnostics": sorted(self.diagnostics),
            "final_position_m": tuple(round(value, 2) for value in self.position()),
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENARIOS, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Scenario(args.scenario)
    try:
        summary = node.run()
    except Exception as error:  # one explicit scenario boundary
        summary = {
            "scenario": args.scenario,
            "result": "FAIL",
            "error": f"{type(error).__name__}: {error}",
            "diagnostics": sorted(node.diagnostics),
            "final_position_m": tuple(round(value, 2) for value in node.position()),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.summary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))
    raise SystemExit(0 if summary["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
