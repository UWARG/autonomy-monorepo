#!/usr/bin/env python3
"""Exercise the #232 position controller with 1 Hz moving traffic in SITL."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from airside_interfaces.msg import (
    Coordinate,
    PositionTarget,
    TrafficAircraft,
    TrafficSnapshot,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from mock_aeac_websocket import MockAeacWebSocket

EARTH_RADIUS_M = 6_371_000.0
ALTITUDE_M = 15.0
GOAL_NORTH_M = 40.0
SCENARIOS = (
    "clear",
    "moving",
    "multi",
    "turn",
    "dropout",
    "blocked",
    "pilot",
    "sharp_turn",
    "triangle15",
    "triangle20",
)
STRESS_SCENARIOS = {"sharp_turn", "triangle15", "triangle20"}


def offset(
    lat: float, lon: float, east_m: float, north_m: float
) -> tuple[float, float]:
    return (
        lat + math.degrees(north_m / EARTH_RADIUS_M),
        lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))),
    )


class Scenario(Node):
    def __init__(self, name: str, traffic_source: str = "direct") -> None:
        super().__init__("moving_traffic_sitl_readiness")
        self.scenario = name
        self.traffic_source = traffic_source
        self.state: State | None = None
        self.fix: NavSatFix | None = None
        self.altitude_m: float | None = None
        self.pose: PoseStamped | None = None
        self.home_pose: tuple[float, float] | None = None
        self.home_wgs84: tuple[float, float] | None = None
        self.traffic_started_s: float | None = None
        self.target_enabled = False
        self.traffic_enabled = False
        self.sequence = time.time_ns()
        self.setpoint_count = 0
        self.status_reasons: set[str] = set()
        self.hold_reasons: set[str] = set()
        self.max_traffic_age_s = 0.0
        self.max_expanded_radii: dict[int, dict[str, float]] = {}
        self.alerts: list[str] = []
        self.min_raw_clearance_m = math.inf
        self.min_raw_clearance_event: dict[str, object] | None = None
        self.max_observed_speed_mps = 0.0
        self.current_observed_speed_mps = 0.0
        self.hold_started_s: float | None = None
        self.hold_start_speed_mps = 0.0
        self.measured_stop_s: float | None = None
        self.track: deque[tuple[float, float, float]] = deque()
        self.listener_snapshots = 0
        self.listener_indices: set[int] = set()
        self.listener_healthy = False
        self.mock_server: MockAeacWebSocket | None = None
        self.configured_speed_mps: float | None = None
        self.setpoint_publishers: list[tuple[str, str]] = []

        self.create_subscription(State, "/mavros/state", self._state, 10)
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._fix,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self._alt,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", self._setpoint, 10
        )
        self.create_subscription(
            String, "/position_controller/avoidance_status", self._status, 10
        )
        self.create_subscription(StatusText, "/mavros/statustext/send", self._alert, 10)
        self.target_pub = self.create_publisher(
            PositionTarget, "/position_controller/target", 10
        )
        self.traffic_pub = (
            self.create_publisher(TrafficSnapshot, "/aeac/traffic", 10)
            if traffic_source == "direct"
            else None
        )
        if traffic_source == "listener":
            self.create_subscription(
                TrafficSnapshot, "/aeac/traffic", self._listener_snapshot, 10
            )
        self.create_timer(0.2, self.publish_target)
        if traffic_source == "direct":
            self.create_timer(1.0, self.publish_traffic)
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.param_client = self.create_client(
            GetParameters, "/mavros/param/get_parameters"
        )

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
        if not self.target_enabled:
            return
        east_m, north_m = self.position()
        now_s = time.monotonic()
        self.track.append((now_s, east_m, north_m))
        while self.track and now_s - self.track[0][0] > 1.5:
            self.track.popleft()
        for old_s, old_east, old_north in self.track:
            elapsed_s = now_s - old_s
            if 0.8 <= elapsed_s <= 1.2:
                speed_mps = (
                    math.hypot(east_m - old_east, north_m - old_north) / elapsed_s
                )
                self.max_observed_speed_mps = max(
                    self.max_observed_speed_mps, speed_mps
                )
                self.current_observed_speed_mps = speed_mps
                if (
                    self.hold_started_s is not None
                    and self.hold_start_speed_mps >= 0.5
                    and self.measured_stop_s is None
                    and speed_mps <= 0.2
                ):
                    self.measured_stop_s = now_s - self.hold_started_s
                break
        for index, obstacle_east, obstacle_north, _, _, keepaway_m in self.aircraft(
            now_s
        ):
            clearance_m = (
                math.hypot(east_m - obstacle_east, north_m - obstacle_north)
                - keepaway_m
            )
            if clearance_m < self.min_raw_clearance_m:
                self.min_raw_clearance_m = clearance_m
                self.min_raw_clearance_event = {
                    "aircraft_index": index,
                    "elapsed_s": now_s - (self.traffic_started_s or now_s),
                    "own_east_north_m": [east_m, north_m],
                    "traffic_east_north_m": [obstacle_east, obstacle_north],
                    "raw_keepaway_m": keepaway_m,
                }

    def _setpoint(self, _: GlobalPositionTarget) -> None:
        self.setpoint_count += 1

    def _status(self, msg: String) -> None:
        payload = json.loads(msg.data)
        self.status_reasons.add(payload["reason"])
        if payload.get("hold_reason"):
            self.hold_reasons.add(payload["hold_reason"])
        if payload["reason"] == "PATH_FOUND":
            self.hold_started_s = None
        elif self.hold_started_s is None:
            self.hold_started_s = time.monotonic()
            self.hold_start_speed_mps = self.current_observed_speed_mps
        age_s = payload.get("traffic_age_s")
        if isinstance(age_s, (int, float)) and math.isfinite(age_s):
            self.max_traffic_age_s = max(self.max_traffic_age_s, age_s)
        for radius in payload.get("radii", []):
            index = int(radius["aircraft_index"])
            current = self.max_expanded_radii.get(index)
            expanded_m = float(radius["expanded_radius_m"])
            if current is None or expanded_m > current["max_expanded_radius_m"]:
                self.max_expanded_radii[index] = {
                    "raw_radius_m": float(radius["raw_radius_m"]),
                    "max_expanded_radius_m": expanded_m,
                }

    def _alert(self, msg: StatusText) -> None:
        self.alerts.append(msg.text)

    def _listener_snapshot(self, msg: TrafficSnapshot) -> None:
        self.listener_snapshots += 1
        self.listener_healthy = msg.connected and msg.healthy
        self.listener_indices.update(item.aircraft_index for item in msg.aircraft)
        if msg.own_aircraft_index != 1:
            raise RuntimeError(
                f"listener own aircraft index was {msg.own_aircraft_index}, expected 1"
            )
        if any(item.aircraft_index == 1 for item in msg.aircraft):
            raise RuntimeError("listener did not filter own aircraft")

    def mock_event(self) -> dict[str, object] | None:
        if self.home_wgs84 is None:
            return None
        entries = []
        for index, east_m, north_m, speed_mps, heading_deg, keepaway_m in [
            (1, 0.0, 0.0, 0.0, 0.0, 5.0),
            *self.aircraft(time.monotonic()),
        ]:
            lat, lon = offset(*self.home_wgs84, east_m, north_m)
            entries.append(
                {
                    "aircraftIndex": index,
                    "name": f"SITL-{index}",
                    "horizontalKeepAway": keepaway_m,
                    "verticalKeepAway": 5.0,
                    "position": {
                        "lat": lat,
                        "lon": lon,
                        "altitude": 30.0,
                        "speed": speed_mps,
                        "direction": heading_deg,
                    },
                }
            )
        return {"event": "traffic", "payload": {"traffic": entries}}

    def position(self) -> tuple[float, float]:
        if self.pose is None or self.home_pose is None:
            return 0.0, 0.0
        return (
            self.pose.pose.position.x - self.home_pose[0],
            self.pose.pose.position.y - self.home_pose[1],
        )

    def aircraft(
        self, now_s: float
    ) -> list[tuple[int, float, float, float, float, float]]:
        if self.scenario in STRESS_SCENARIOS:
            elapsed_s = max(0.0, now_s - (self.traffic_started_s or now_s))
            speed_mps = {
                "sharp_turn": 5.0,
                "triangle15": 15.0,
                "triangle20": 20.0,
            }[self.scenario]
            corners = ((-30.0, 20.0), (30.0, 20.0), (0.0, 50.0))
            lengths = [math.dist(corners[i], corners[(i + 1) % 3]) for i in range(3)]
            distance_m = (elapsed_s * speed_mps) % sum(lengths)
            for i, length_m in enumerate(lengths):
                if distance_m <= length_m:
                    start, end = corners[i], corners[(i + 1) % 3]
                    fraction = distance_m / length_m
                    east_m = start[0] + fraction * (end[0] - start[0])
                    north_m = start[1] + fraction * (end[1] - start[1])
                    heading_deg = (
                        math.degrees(math.atan2(end[0] - start[0], end[1] - start[1]))
                        % 360.0
                    )
                    return [(2, east_m, north_m, speed_mps, heading_deg, 5.0)]
                distance_m -= length_m
            raise AssertionError("triangle segment not found")
        if self.scenario == "blocked":
            return [
                (
                    2 + i,
                    30.0 * math.sin(math.radians(i * 45)),
                    30.0 * math.cos(math.radians(i * 45)),
                    0.0,
                    0.0,
                    20.0,
                )
                for i in range(8)
            ]
        if self.scenario not in ("moving", "multi", "turn"):
            return []
        elapsed_s = max(0.0, now_s - (self.traffic_started_s or now_s))
        if self.scenario == "turn" and elapsed_s > 6.0:
            first = (
                2,
                -15.0 + 1.2 * 6.0,
                20.0 + 1.2 * (elapsed_s - 6.0),
                1.2,
                0.0,
                5.0,
            )
        else:
            first = (2, -15.0 + 1.2 * elapsed_s, 20.0, 1.2, 90.0, 5.0)
        if self.scenario == "multi":
            second = (3, 15.0 - 1.2 * elapsed_s, 28.0, 1.2, 270.0, 5.0)
            return [first, second]
        return [first]

    def publish_target(self) -> None:
        if not self.target_enabled or self.home_wgs84 is None:
            return
        lat, lon = offset(*self.home_wgs84, 0.0, GOAL_NORTH_M)
        self.target_pub.publish(
            PositionTarget(target=Coordinate(lat=lat, lon=lon, alt=ALTITUDE_M))
        )

    def publish_traffic(self) -> None:
        if (
            not self.traffic_enabled
            or self.home_wgs84 is None
            or self.traffic_pub is None
        ):
            return
        now_s = time.monotonic()
        self.sequence += 1
        snapshot = TrafficSnapshot()
        snapshot.header.stamp = self.get_clock().now().to_msg()
        snapshot.header.frame_id = "wgs84"
        snapshot.sequence = self.sequence
        snapshot.connected = True
        snapshot.healthy = True
        snapshot.own_aircraft_index = 1
        for index, east_m, north_m, speed_mps, heading_deg, keepaway_m in self.aircraft(
            now_s
        ):
            lat, lon = offset(*self.home_wgs84, east_m, north_m)
            snapshot.aircraft.append(
                TrafficAircraft(
                    aircraft_index=index,
                    name=f"SITL-{index}",
                    latitude_deg=lat,
                    longitude_deg=lon,
                    altitude_agl_m=30.0,  # Conservative live mode must still include it.
                    speed_mps=speed_mps,
                    heading_deg_true=heading_deg,
                    horizontal_keepaway_m=keepaway_m,
                    vertical_keepaway_m=5.0,
                )
            )
        self.traffic_pub.publish(snapshot)

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
                self.wait_for(
                    "armed state",
                    lambda: self.state is not None and self.state.armed,
                    10.0,
                )
                return
            self.pump(1.0)
        raise RuntimeError("FCU did not arm")

    def check_speed_parameter(self) -> float:
        response = self.call(
            self.param_client, GetParameters.Request(names=["WPNAV_SPEED"])
        )
        if len(response.values) != 1:
            raise RuntimeError("WPNAV_SPEED missing from SITL")
        value = response.values[0]
        if value.type == ParameterType.PARAMETER_INTEGER:
            raw_value = float(value.integer_value)
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            raw_value = value.double_value
        else:
            raise RuntimeError("WPNAV_SPEED has unsupported type")
        speed_mps = raw_value / 100.0
        if not 1.49 <= speed_mps <= 1.51:
            raise RuntimeError(f"WPNAV_SPEED readback was {speed_mps:.2f} m/s")
        return speed_mps

    def run(self) -> dict[str, object]:
        self.wait_for(
            "connected FCU and GPS fix",
            lambda: (
                self.state is not None
                and self.state.connected
                and self.fix is not None
                and self.fix.status.status >= NavSatStatus.STATUS_FIX
                and self.pose is not None
            ),
            120.0,
        )
        self.wait_for(
            "position controller publisher",
            lambda: bool(
                self.get_publishers_info_by_topic("/mavros/setpoint_raw/global")
            ),
            15.0,
        )
        owners = [
            (item.node_name, item.node_namespace)
            for item in self.get_publishers_info_by_topic("/mavros/setpoint_raw/global")
        ]
        self.setpoint_publishers = owners
        if owners != [("position_controller", "/")]:
            raise RuntimeError(f"unexpected MAVROS setpoint publishers: {owners}")
        configured_speed_mps = self.check_speed_parameter()
        self.configured_speed_mps = configured_speed_mps
        assert self.fix is not None
        self.home_wgs84 = (self.fix.latitude, self.fix.longitude)
        self.set_mode("GUIDED")
        self.arm()
        response = self.call(
            self.takeoff_client, CommandTOL.Request(altitude=ALTITUDE_M)
        )
        if not response.success:
            raise RuntimeError(f"takeoff rejected: {response.result}")
        self.wait_for(
            "takeoff altitude",
            lambda: self.altitude_m is not None and self.altitude_m >= ALTITUDE_M - 2.0,
            70.0,
        )
        self.traffic_started_s = time.monotonic()
        self.traffic_enabled = True
        if self.traffic_source == "listener":
            self.mock_server = MockAeacWebSocket(self.mock_event)
            self.mock_server.start()
            self.wait_for(
                "healthy listener snapshot",
                lambda: (
                    self.listener_healthy
                    and (self.scenario != "moving" or 2 in self.listener_indices)
                ),
                15.0,
            )
            owners = [
                item.node_name
                for item in self.get_publishers_info_by_topic("/aeac/traffic")
            ]
            if owners != ["traffic_listener"]:
                raise RuntimeError(f"unexpected traffic publishers: {owners}")
        else:
            self.publish_traffic()
        self.target_enabled = True
        self.publish_target()

        if self.scenario == "blocked":
            self.wait_for(
                "blocked hold", lambda: "BLOCKED" in self.status_reasons, 10.0
            )
            self.wait_for("pilot alert", lambda: bool(self.alerts), 10.0)
            self.pump(1.0)
            if math.hypot(*self.position()) > 2.0:
                raise RuntimeError("vehicle moved into a blocked route")
        elif self.scenario == "dropout":
            self.wait_for("initial movement", lambda: self.position()[1] >= 5.0, 35.0)
            dropout_at = self.position()
            if self.traffic_source == "listener":
                assert self.mock_server is not None
                self.mock_server.close()
                self.mock_server = None
            else:
                self.traffic_enabled = False
            self.wait_for(
                "stale traffic hold",
                lambda: (
                    "STALE_TRAFFIC" in self.status_reasons
                    or "STALE_TRAFFIC" in self.hold_reasons
                    or "AEAC_DISCONNECTED" in self.status_reasons
                    or "AEAC_DISCONNECTED" in self.hold_reasons
                ),
                7.0,
            )
            self.pump(2.0)
            if math.dist(dropout_at, self.position()) > 5.0:
                raise RuntimeError("vehicle progressed too far after traffic loss")
            if self.traffic_source == "listener":
                self.mock_server = MockAeacWebSocket(self.mock_event)
                self.mock_server.start()
                self.wait_for(
                    "reconnected listener", lambda: self.listener_healthy, 15.0
                )
            else:
                self.traffic_enabled = True
                self.publish_traffic()
            self.wait_for("resumed goal", lambda: self.position()[1] >= 38.0, 90.0)
        elif self.scenario == "pilot":
            self.wait_for("initial movement", lambda: self.position()[1] >= 5.0, 35.0)
            self.set_mode("LOITER")
            self.target_enabled = False
            self.pump(0.3)
            count_at_takeover = self.setpoint_count
            self.pump(1.5)
            if self.setpoint_count != count_at_takeover:
                raise RuntimeError("setpoints continued after pilot takeover")
            self.set_mode("GUIDED")
            self.pump(0.8)
            if self.setpoint_count != count_at_takeover:
                raise RuntimeError("old target replayed after returning to GUIDED")
            self.target_enabled = True
            self.publish_target()
            self.wait_for("resumed goal", lambda: self.position()[1] >= 38.0, 90.0)
        elif self.scenario in STRESS_SCENARIOS:
            # This is a safety stress test, not a flight-route qualification.
            # A conservative controller may remain in hold for the whole loop.
            self.pump(25.0)
            if not self.status_reasons:
                raise RuntimeError("controller emitted no avoidance status")
            if self.scenario in ("triangle15", "triangle20") and (
                "TRAFFIC_SPEED_OUT_OF_ENVELOPE" not in self.hold_reasons
            ):
                raise RuntimeError("high-speed traffic did not trigger the speed hold")
            if self.position()[1] < 38.0 and not self.alerts:
                raise RuntimeError(
                    "stress route did not complete and pilot was not alerted"
                )
        else:
            self.wait_for("goal", lambda: self.position()[1] >= 38.0, 110.0)

        if self.min_raw_clearance_m < 3.0:
            raise RuntimeError(
                f"raw keep-away margin below 3 m: {self.min_raw_clearance_m:.2f}"
            )
        if self.max_observed_speed_mps > 1.7:
            raise RuntimeError(
                f"observed speed exceeded 1.7 m/s peak: {self.max_observed_speed_mps:.2f}"
            )
        return {
            "scenario": self.scenario,
            "traffic_source": self.traffic_source,
            "result": "PASS_STRESS_ONLY"
            if self.scenario in STRESS_SCENARIOS
            else "PASS",
            "listener_snapshots": self.listener_snapshots,
            "listener_indices": sorted(self.listener_indices),
            "configured_speed_mps": configured_speed_mps,
            "setpoint_publishers": self.setpoint_publishers,
            "max_observed_speed_mps": round(self.max_observed_speed_mps, 3),
            "measured_stop_s": self.measured_stop_s,
            "min_raw_clearance_m": None
            if math.isinf(self.min_raw_clearance_m)
            else round(self.min_raw_clearance_m, 3),
            "min_raw_clearance_event": self.min_raw_clearance_event,
            "setpoint_count": self.setpoint_count,
            "status_reasons": sorted(self.status_reasons),
            "hold_reasons": sorted(self.hold_reasons),
            "max_traffic_age_s": round(self.max_traffic_age_s, 3),
            "max_expanded_radii": self.max_expanded_radii,
            "pilot_alerts": self.alerts,
            "final_position_m": tuple(round(value, 2) for value in self.position()),
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=SCENARIOS, required=True)
    parser.add_argument(
        "--traffic-source", choices=("direct", "listener"), default="direct"
    )
    parser.add_argument("--summary", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Scenario(args.scenario, args.traffic_source)
    try:
        result = node.run()
    except Exception as error:
        result = {
            "scenario": args.scenario,
            "traffic_source": args.traffic_source,
            "result": "FAIL",
            "error": f"{type(error).__name__}: {error}",
            "status_reasons": sorted(node.status_reasons),
            "hold_reasons": sorted(node.hold_reasons),
            "max_traffic_age_s": round(node.max_traffic_age_s, 3),
            "max_expanded_radii": node.max_expanded_radii,
            "min_raw_clearance_m": node.min_raw_clearance_m,
            "min_raw_clearance_event": node.min_raw_clearance_event,
            "listener_snapshots": node.listener_snapshots,
            "listener_indices": sorted(node.listener_indices),
            "max_observed_speed_mps": node.max_observed_speed_mps,
            "measured_stop_s": node.measured_stop_s,
            "setpoint_count": node.setpoint_count,
            "configured_speed_mps": node.configured_speed_mps,
            "setpoint_publishers": node.setpoint_publishers,
            "final_position_m": tuple(round(value, 2) for value in node.position()),
        }
    finally:
        if node.mock_server is not None:
            node.mock_server.close()
        node.destroy_node()
        rclpy.shutdown()
    args.summary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result))
    raise SystemExit(0 if result["result"].startswith("PASS") else 1)


if __name__ == "__main__":
    main()
