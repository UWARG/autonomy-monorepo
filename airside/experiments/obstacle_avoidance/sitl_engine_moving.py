#!/usr/bin/env python3
"""Run the real mission manager, listener and controller against moving SITL traffic."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from airside_interfaces.msg import PositionTarget, WaypointAdjustment, WaypointList
from obstacle_avoidance.boundary import FlightBoundary
from obstacle_avoidance.waypoint_relocation import replacement_geometry_valid
from utils.src.types import Coordinate

from server_traffic_replay import load_route, route_length_m
from sitl_moving_readiness import MockAeacWebSocket, Scenario
from utils.src.waypoint_utils import east_north_coordinate_offset_m


class EngineScenario(Scenario):
    def __init__(self, route: Path, scenario: str = "engine-eight") -> None:
        super().__init__("moving", traffic_source="listener")
        self.scenario_name = scenario
        self.replacement_mode = scenario == "engine-replace"
        # The mission manager, not this diagnostic node, must own the target topic.
        self.destroy_publisher(self.target_pub)
        self.route = load_route(route)
        self.route_file = route
        self.engine_waypoints: list[tuple[float, float]] = []
        self.target_sequence: list[tuple[float, float]] = []
        self.target_path_lengths: list[int] = []
        self.target_path_error: str | None = None
        self.first_leg_target: tuple[float, float] | None = None
        self.replacement_proposals: list[tuple[int, float, float]] = []
        self.replacement_targets: list[tuple[float, float]] = []
        self.replacement_completion_distance_m: float | None = None
        self.crossing_started_s: float | None = None
        self.crossing_center: tuple[float, float] | None = None
        self.crossing_direction: tuple[float, float] | None = None
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

    def _adjustment(self, msg: WaypointAdjustment) -> None:
        self.replacement_proposals.append(
            (msg.leg_id, msg.replacement.lat, msg.replacement.lon)
        )

    def publish_target(self) -> None:
        """EngineScenario never injects a target; the engine publishes it."""

    def _engine_waypoints(self, msg: WaypointList) -> None:
        self.engine_waypoints = [(p.lat, p.lon) for p in msg.waypoints]

    def _engine_target(self, msg: PositionTarget) -> None:
        point = (msg.target.lat, msg.target.lon)
        if not self.target_sequence or point != self.target_sequence[-1]:
            if (
                self.replacement_targets
                and self.target_sequence
                and self._same_point(
                    self.target_sequence[-1], self.replacement_targets[-1]
                )
                and self.fix is not None
            ):
                self.replacement_completion_distance_m = math.hypot(
                    *east_north_coordinate_offset_m(
                        self.fix.latitude,
                        self.fix.longitude,
                        *self.replacement_targets[-1],
                    )
                )
            self.target_sequence.append(point)
            self.target_path_lengths.append(len(msg.path))
        if msg.leg_id and msg.route_vertices:
            original = msg.route_vertices[msg.route_index]
            if not self._same_point(point, (original.lat, original.lon)):
                if not self.replacement_targets or not self._same_point(
                    point, self.replacement_targets[-1]
                ):
                    self.replacement_targets.append(point)
        if len(msg.path) < 2 or msg.target_index >= len(msg.path):
            self.target_path_error = "engine target lacks a usable path"
            return
        through = msg.path[msg.target_index]
        if (through.lat, through.lon, through.alt) != (
            msg.target.lat,
            msg.target.lon,
            msg.target.alt,
        ):
            self.target_path_error = "path[target_index] does not equal target"
            return
        if self.home_wgs84 is None or self.first_leg_target is not None:
            return
        east_m, north_m = east_north_coordinate_offset_m(*self.home_wgs84, *point)
        length_m = math.hypot(east_m, north_m)
        if length_m < 5.0:
            return  # The first route waypoint is at the SITL home position.
        along_east, along_north = east_m / length_m, north_m / length_m
        self.first_leg_target = point
        self.crossing_center = (east_m / 2.0, north_m / 2.0)
        self.crossing_direction = (-along_north, along_east)
        self.crossing_started_s = time.monotonic()

    def aircraft(
        self, now_s: float
    ) -> list[tuple[int, float, float, float, float, float]]:
        if self.replacement_mode:
            if len(self.engine_waypoints) < 2 or self.home_wgs84 is None:
                return []
            center_e, center_n = east_north_coordinate_offset_m(
                *self.home_wgs84, *self.engine_waypoints[1]
            )
            elapsed_s = max(0.0, now_s - (self.traffic_started_s or now_s))
            phase = elapsed_s * 0.2
            east_m = center_e + 2.0 * math.cos(phase)
            north_m = center_n + 2.0 * math.sin(phase)
            heading = (
                math.degrees(math.atan2(-math.sin(phase), math.cos(phase))) % 360.0
            )
            return [(2, east_m, north_m, 0.4, heading, 2.0)]
        if self.crossing_started_s is None:
            return []
        assert self.crossing_center is not None
        assert self.crossing_direction is not None
        elapsed_s = max(0.0, now_s - self.crossing_started_s)
        lateral_m = -12.0 + 1.2 * elapsed_s
        direction_east, direction_north = self.crossing_direction
        east_m = self.crossing_center[0] + direction_east * lateral_m
        north_m = self.crossing_center[1] + direction_north * lateral_m
        heading_deg = math.degrees(math.atan2(direction_east, direction_north)) % 360.0
        return [(2, east_m, north_m, 1.2, heading_deg, 5.0)]

    def _same_point(
        self, left: tuple[float, float], right: tuple[float, float]
    ) -> bool:
        return math.hypot(*east_north_coordinate_offset_m(*left, *right)) < 0.5

    def _lap_targets_seen(self) -> bool:
        expected = self.engine_waypoints
        actual = self.target_sequence
        if len(expected) != len(self.route) or len(actual) < len(expected) + 1:
            return False
        if self.replacement_mode:
            if not self.replacement_targets:
                return False
            cursor = 0
            for point in actual:
                if cursor < len(expected) + 1 and self._same_point(
                    point, [*expected, expected[0]][cursor]
                ):
                    cursor += 1
            return cursor == len(expected) + 1
        return all(
            self._same_point(got, want)
            for got, want in zip(actual, [*expected, expected[0]])
        )

    def summary(self, result: str, error: Exception | None = None) -> dict[str, object]:
        payload: dict[str, object] = {
            "scenario": self.scenario_name,
            "result": result,
            "route_file": str(self.route_file),
            "route_waypoints": len(self.route),
            "route_length_m": round(route_length_m(self.route), 3),
            "engine_waypoints": self.engine_waypoints,
            "target_sequence": self.target_sequence,
            "target_path_lengths": self.target_path_lengths,
            "target_path_error": self.target_path_error,
            "replacement_proposals": self.replacement_proposals,
            "replacement_targets": self.replacement_targets,
            "replacement_completion_distance_m": self.replacement_completion_distance_m,
            "first_leg_target": self.first_leg_target,
            "listener_snapshots": self.listener_snapshots,
            "listener_indices": sorted(self.listener_indices),
            "configured_speed_mps": self.configured_speed_mps,
            "setpoint_publishers": self.setpoint_publishers,
            "setpoint_count": self.setpoint_count,
            "max_observed_speed_mps": round(self.max_observed_speed_mps, 3),
            "min_raw_clearance_m": None
            if math.isinf(self.min_raw_clearance_m)
            else round(self.min_raw_clearance_m, 3),
            "min_raw_clearance_event": self.min_raw_clearance_event,
            "max_traffic_age_s": round(self.max_traffic_age_s, 3),
            "max_expanded_radii": self.max_expanded_radii,
            "status_reasons": sorted(self.status_reasons),
            "pilot_alerts": self.alerts,
            "final_position_m": tuple(round(value, 2) for value in self.position()),
            "rosbag": "rosbag/",
        }
        if error is not None:
            payload["error"] = f"{type(error).__name__}: {error}"
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
        if not self._same_point(self.home_wgs84, self.route[0]):
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
        self.mock_server = MockAeacWebSocket(self.mock_event)
        self.mock_server.start()
        self.wait_for("healthy listener snapshot", lambda: self.listener_healthy, 15.0)
        self.set_mode("GUIDED")
        self.arm()
        self.target_enabled = True
        self.wait_for(
            "engine-loaded waypoints",
            lambda: len(self.engine_waypoints) == len(self.route),
            30.0,
        )
        self.wait_for(
            "engine moving path target",
            lambda: self.first_leg_target is not None,
            90.0,
        )
        self.wait_for(
            "listener moving aircraft",
            lambda: 2 in self.listener_indices,
            10.0,
        )
        self.wait_for(
            "complete engine lap target sequence",
            self._lap_targets_seen,
            min(600.0, route_length_m(self.route) / 1.2 + 160.0),
        )
        assert self.engine_waypoints
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
        owners = [
            item.node_name
            for item in self.get_publishers_info_by_topic("/position_controller/target")
        ]
        # Multiple engine behaviors create endpoints on the same node. Only a
        # different node would be a competing target publisher.
        if not owners or set(owners) != {"engine_manager"}:
            raise RuntimeError(f"unexpected target publishers: {owners}")
        if self.target_path_error is not None:
            raise RuntimeError(self.target_path_error)
        if self.replacement_mode:
            if not self.replacement_proposals or not self.replacement_targets:
                raise RuntimeError("blocked waypoint was not replaced")
            if (
                self.replacement_completion_distance_m is None
                or self.replacement_completion_distance_m > 3.0
            ):
                raise RuntimeError("replacement waypoint was not reached within 3 m")
            route = tuple(
                Coordinate(lat, lon, 10.0) for lat, lon in self.engine_waypoints
            )
            fence = FlightBoundary.from_json(os.environ["FLIGHT_BOUNDARY_WGS84"])
            previous_shift_m = 0.0
            for point in self.replacement_targets:
                candidate = Coordinate(*point, 10.0)
                if not replacement_geometry_valid(route[1], candidate, route, 1, fence):
                    raise RuntimeError("replacement lies inside route or outside fence")
                shift_m = math.hypot(
                    *east_north_coordinate_offset_m(route[1].lat, route[1].lon, *point)
                )
                if shift_m <= previous_shift_m + 0.5:
                    raise RuntimeError("replacement target flickered or moved inward")
                previous_shift_m = shift_m
        if self.listener_snapshots < 3 or 2 not in self.listener_indices:
            raise RuntimeError("moving traffic was not delivered by the listener")
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
    parser.add_argument("--route", required=True, type=Path)
    parser.add_argument("--scenario", default="engine-eight")
    parser.add_argument("--summary", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node: EngineScenario | None = None
    try:
        node = EngineScenario(args.route, args.scenario)
        result = node.run()
    except Exception as error:
        result = (
            node.summary("FAIL", error)
            if node is not None
            else {"scenario": "engine-moving", "result": "FAIL", "error": str(error)}
        )
    finally:
        if node is not None:
            if node.mock_server is not None:
                node.mock_server.close()
            node.destroy_node()
        rclpy.shutdown()
    args.summary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result))
    raise SystemExit(0 if result["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
