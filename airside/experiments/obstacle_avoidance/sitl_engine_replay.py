#!/usr/bin/env python3
"""Replay captured traffic through the real mission manager in SITL."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from airside_interfaces.msg import PositionTarget, WaypointAdjustment, WaypointList
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from server_traffic_replay import route_length_m
from sitl_server_replay import ReplayScenario


class EngineReplayScenario(ReplayScenario):
    def __init__(
        self, capture: Path, route: Path, own_index: int, replay_start_s: float
    ) -> None:
        super().__init__(capture, route, own_index, replay_start_s)
        self.destroy_publisher(self.target_pub)
        self.engine_waypoints: list[tuple[float, float]] = []
        self.target_sequence: list[tuple[float, float]] = []
        self.target_leg_id: int | None = None
        self.leg_completion_distances_m: list[float] = []
        self.target_path_error: str | None = None
        self.replacement_proposals: list[tuple[int, float, float]] = []
        self.replacement_targets: list[tuple[float, float]] = []
        self.create_subscription(
            WaypointList, "/engine/waypoints", self._engine_waypoints, 10
        )
        self.create_subscription(
            PositionTarget, "/position_controller/target", self._engine_target, 10
        )
        self.create_subscription(
            WaypointAdjustment,
            "/position_controller/waypoint_adjustment",
            self._adjustment,
            10,
        )

    def publish_target(self) -> None:
        """Only the mission manager may publish targets in this test."""

    def _engine_waypoints(self, msg: WaypointList) -> None:
        self.engine_waypoints = [(p.lat, p.lon) for p in msg.waypoints]

    def _adjustment(self, msg: WaypointAdjustment) -> None:
        self.replacement_proposals.append(
            (msg.leg_id, msg.replacement.lat, msg.replacement.lon)
        )

    def _engine_target(self, msg: PositionTarget) -> None:
        point = (msg.target.lat, msg.target.lon)
        if not msg.leg_id or not msg.route_vertices:
            self.target_path_error = "engine target lacks route identity"
        if (
            self.target_leg_id is not None
            and msg.leg_id != self.target_leg_id
            and self.target_sequence
            and self.fix is not None
        ):
            self.leg_completion_distances_m.append(
                math.hypot(
                    *east_north_coordinate_offset_m(
                        self.fix.latitude,
                        self.fix.longitude,
                        *self.target_sequence[-1],
                    )
                )
            )
        self.target_leg_id = msg.leg_id
        if not self.target_sequence or point != self.target_sequence[-1]:
            self.target_sequence.append(point)
        if len(msg.path) < 2 or msg.target_index >= len(msg.path):
            self.target_path_error = "engine target lacks a usable path"
        elif (msg.path[msg.target_index].lat, msg.path[msg.target_index].lon) != point:
            self.target_path_error = "path[target_index] does not equal target"
        if msg.leg_id and msg.route_vertices:
            original = msg.route_vertices[msg.route_index]
            if (
                math.hypot(
                    *east_north_coordinate_offset_m(original.lat, original.lon, *point)
                )
                > 0.5
                and point not in self.replacement_targets
            ):
                self.replacement_targets.append(point)

    def _lap_complete(self) -> bool:
        if len(self.engine_waypoints) != len(self.route):
            return False
        expected = [*self.engine_waypoints, self.engine_waypoints[0]]
        cursor = 0
        for point in self.target_sequence:
            if (
                cursor < len(expected)
                and math.hypot(
                    *east_north_coordinate_offset_m(*point, *expected[cursor])
                )
                < 0.5
            ):
                cursor += 1
        return cursor == len(expected)

    def summary(self, result: str, error: Exception | None = None) -> dict[str, object]:
        payload = super().summary(result, error)
        payload.update(
            scenario="engine-replay",
            engine_waypoints=self.engine_waypoints,
            target_sequence=self.target_sequence,
            leg_completion_distances_m=self.leg_completion_distances_m,
            target_path_error=self.target_path_error,
            replacement_proposals=self.replacement_proposals,
            replacement_targets=self.replacement_targets,
            capture_exhausted=self.next_frame == len(self.frames),
        )
        return payload

    def run(self) -> dict[str, object]:
        self.wait_for(
            "connected FCU and GPS fix",
            lambda: (
                self.state is not None
                and self.state.connected
                and self.fix is not None
                and self.pose is not None
            ),
            120.0,
        )
        assert self.fix is not None
        self.home_wgs84 = (self.fix.latitude, self.fix.longitude)
        if (
            math.hypot(
                *east_north_coordinate_offset_m(*self.home_wgs84, *self.route[0])
            )
            > 0.5
        ):
            raise RuntimeError("SITL home is not at the first route waypoint")
        self.wait_for(
            "sole setpoint publisher",
            lambda: bool(
                self.get_publishers_info_by_topic("/mavros/setpoint_raw/global")
            ),
            15.0,
        )
        self.setpoint_publishers = [
            (item.node_name, item.node_namespace)
            for item in self.get_publishers_info_by_topic("/mavros/setpoint_raw/global")
        ]
        if self.setpoint_publishers != [("position_controller", "/")]:
            raise RuntimeError(
                f"unexpected MAVROS setpoint publishers: {self.setpoint_publishers}"
            )
        self.configured_speed_mps = self.check_speed_parameter()
        self.traffic_started_s = time.monotonic()
        self.traffic_enabled = True
        self.publish_traffic()
        self.set_mode("GUIDED")
        self.arm()
        self.target_enabled = True
        self.wait_for(
            "engine-loaded waypoints",
            lambda: len(self.engine_waypoints) == len(self.route),
            30.0,
        )
        if any(
            math.hypot(*east_north_coordinate_offset_m(*loaded, *file_point)) > 0.5
            for loaded, file_point in zip(self.engine_waypoints, self.route)
        ):
            raise RuntimeError("mission manager reordered the replay route")
        lap_deadline_s = time.monotonic() + min(
            300.0, route_length_m(self.route) / 1.2 + 120.0
        )
        while not self._lap_complete():
            if (
                self.next_frame == len(self.frames)
                and time.monotonic() - (self.traffic_started_s or time.monotonic())
                > self.frames[-1].time_s - self.start_s + 1.5
            ):
                raise RuntimeError("CAPTURE_EXHAUSTED before lap completion")
            if time.monotonic() >= lap_deadline_s:
                raise TimeoutError("timed out waiting for complete engine lap")
            rclpy.spin_once(self, timeout_sec=0.2)
        closure = self.engine_waypoints[0]
        self.wait_for(
            "lap closure position",
            lambda: (
                self.fix is not None
                and math.hypot(
                    *east_north_coordinate_offset_m(
                        self.fix.latitude, self.fix.longitude, *closure
                    )
                )
                <= 3.0
            ),
            45.0,
        )
        owners = {
            item.node_name
            for item in self.get_publishers_info_by_topic("/position_controller/target")
        }
        if owners != {"engine_manager"}:
            raise RuntimeError(f"unexpected target publishers: {sorted(owners)}")
        if self.target_path_error:
            raise RuntimeError(self.target_path_error)
        if len(self.leg_completion_distances_m) < len(self.route) or any(
            distance_m > 3.0 for distance_m in self.leg_completion_distances_m
        ):
            raise RuntimeError("mission manager advanced a leg before 3 m arrival")
        if self.published_frames < 3 or not math.isfinite(
            self.candidate_min_raw_clearance_m
        ):
            raise RuntimeError("moving captured traffic was not replayed")
        if self.min_raw_clearance_m < 3.0:
            raise RuntimeError(
                f"raw keep-away margin below 3 m: {self.min_raw_clearance_m:.2f}"
            )
        if self.max_observed_speed_mps > 1.7:
            raise RuntimeError(
                f"observed speed exceeded 1.7 m/s: {self.max_observed_speed_mps:.2f}"
            )
        return self.summary("PASS")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", required=True, type=Path)
    parser.add_argument("--route", required=True, type=Path)
    parser.add_argument("--own-aircraft-index", required=True, type=int)
    parser.add_argument("--replay-start-s", required=True, type=float)
    parser.add_argument("--summary", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node: EngineReplayScenario | None = None
    try:
        node = EngineReplayScenario(
            args.capture, args.route, args.own_aircraft_index, args.replay_start_s
        )
        result = node.run()
    except Exception as error:
        result = {
            "scenario": "engine-replay",
            "result": "FAIL",
            "error": f"{type(error).__name__}: {error}",
        }
        if node is not None:
            result = node.summary("FAIL", error)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    args.summary.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, allow_nan=False))
    raise SystemExit(0 if result["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
