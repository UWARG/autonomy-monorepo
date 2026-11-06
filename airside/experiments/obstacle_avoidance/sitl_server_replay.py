#!/usr/bin/env python3
"""Replay captured AEAC snapshots over a matching route leg in ArduCopter SITL."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from airside_interfaces.msg import (
    Coordinate,
    PositionTarget,
    TrafficAircraft,
    TrafficSnapshot,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import NavSatStatus

from server_traffic_replay import (
    east_north,
    load_capture,
    load_route,
    match_route,
    replay_start_s,
    truth_at,
)
from sitl_moving_readiness import ALTITUDE_M, Scenario, offset


class ReplayScenario(Scenario):
    def __init__(self, capture: Path, route: Path, own_index: int) -> None:
        super().__init__("replay")
        if not 0 <= own_index <= 255:
            raise ValueError("confirmed own aircraft index must be 0..255")
        self.frames = load_capture(capture)
        self.route = load_route(route)
        self.match = match_route(self.frames, self.route, own_index=own_index)
        if self.match is None:
            raise ValueError("capture has no moving aircraft on the configured route")
        self.start_s = replay_start_s(self.match, self.frames)
        self.origin = self.route[self.match.segment_index]
        self.goal = self.route[(self.match.segment_index + 1) % len(self.route)]
        self.goal_east_m, self.goal_north_m = east_north(self.origin, self.goal)
        self.own_index = own_index
        self.indices = {
            aircraft.index for frame in self.frames for aircraft in frame.aircraft
        } - {own_index}
        self.next_frame = next(
            index
            for index, frame in enumerate(self.frames)
            if frame.time_s >= self.start_s
        )
        self.published_frames = 0
        self.publish_times: list[float] = []
        self.candidate_min_raw_clearance_m = math.inf
        self.create_timer(0.05, self.publish_traffic)

    def _pose(self, msg: PoseStamped) -> None:
        super()._pose(msg)
        if not self.target_enabled:
            return
        own_east, own_north = self.position()
        for index, east, north, _, _, radius in self.aircraft(time.monotonic()):
            if index == self.match.aircraft_index:
                self.candidate_min_raw_clearance_m = min(
                    self.candidate_min_raw_clearance_m,
                    math.hypot(own_east - east, own_north - north) - radius,
                )

    def aircraft(
        self, now_s: float
    ) -> list[tuple[int, float, float, float, float, float]]:
        if self.traffic_started_s is None:
            return []
        sample_s = self.start_s + now_s - self.traffic_started_s
        result = []
        for index in self.indices:
            aircraft = truth_at(self.frames, sample_s, index)
            if aircraft is None:
                continue
            east_m, north_m = east_north(self.origin, (aircraft.lat, aircraft.lon))
            result.append(
                (
                    index,
                    east_m,
                    north_m,
                    aircraft.speed_mps,
                    aircraft.heading_deg,
                    aircraft.keepaway_m,
                )
            )
        return result

    def publish_target(self) -> None:
        if not self.target_enabled or self.home_wgs84 is None:
            return
        lat, lon = offset(*self.home_wgs84, self.goal_east_m, self.goal_north_m)
        self.target_pub.publish(
            PositionTarget(target=Coordinate(lat=lat, lon=lon, alt=ALTITUDE_M))
        )

    def publish_traffic(self) -> None:
        if not self.traffic_enabled or self.home_wgs84 is None:
            return
        elapsed_s = time.monotonic() - (self.traffic_started_s or time.monotonic())
        while self.next_frame < len(self.frames):
            frame = self.frames[self.next_frame]
            if frame.time_s - self.start_s > elapsed_s:
                break
            snapshot = TrafficSnapshot()
            snapshot.header.stamp = self.get_clock().now().to_msg()
            snapshot.header.frame_id = "wgs84"
            self.sequence += 1
            snapshot.sequence = self.sequence
            snapshot.connected = True
            snapshot.healthy = True
            snapshot.own_aircraft_index = self.own_index
            for aircraft in frame.aircraft:
                if aircraft.index == self.own_index:
                    continue
                east_m, north_m = east_north(self.origin, (aircraft.lat, aircraft.lon))
                lat, lon = offset(*self.home_wgs84, east_m, north_m)
                snapshot.aircraft.append(
                    TrafficAircraft(
                        aircraft_index=aircraft.index,
                        name="",
                        latitude_deg=lat,
                        longitude_deg=lon,
                        altitude_agl_m=aircraft.altitude_m,
                        speed_mps=aircraft.speed_mps,
                        heading_deg_true=aircraft.heading_deg,
                        horizontal_keepaway_m=aircraft.keepaway_m,
                        vertical_keepaway_m=aircraft.vertical_keepaway_m,
                    )
                )
            self.traffic_pub.publish(snapshot)
            self.publish_times.append(time.monotonic())
            self.published_frames += 1
            self.next_frame += 1

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
        if owners != [("position_controller", "/")]:
            raise RuntimeError(f"unexpected MAVROS setpoint publishers: {owners}")
        configured_speed_mps = self.check_speed_parameter()
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
        self.target_enabled = True
        self.publish_traffic()
        self.publish_target()

        def at_goal() -> bool:
            return (
                math.dist(self.position(), (self.goal_east_m, self.goal_north_m)) <= 2.0
            )

        self.wait_for(
            "route-leg goal",
            at_goal,
            min(180.0, max(60.0, self.match.segment_length_m / 1.5 * 3.0)),
        )
        if self.published_frames < 3:
            raise RuntimeError("fewer than three captured traffic snapshots replayed")
        if not math.isfinite(self.candidate_min_raw_clearance_m):
            raise RuntimeError("matched moving aircraft was absent during replay")
        if self.min_raw_clearance_m < 3.0:
            raise RuntimeError(
                f"raw keep-away margin below 3 m: {self.min_raw_clearance_m:.2f}"
            )
        if self.max_observed_speed_mps > 1.7:
            raise RuntimeError(
                f"observed speed exceeded 1.7 m/s: {self.max_observed_speed_mps:.2f}"
            )
        intervals = [b - a for a, b in zip(self.publish_times, self.publish_times[1:])]
        return {
            "scenario": "replay",
            "result": "PASS",
            "source": "capture_jsonl_replay",
            "aircraft_index": self.match.aircraft_index,
            "route_segment_index": self.match.segment_index,
            "route_centerline_distance_m": round(self.match.centerline_distance_m, 3),
            "replay_start_s": round(self.start_s, 3),
            "replayed_frames": self.published_frames,
            "replay_min_interval_s": min(intervals),
            "replay_max_interval_s": max(intervals),
            "configured_speed_mps": configured_speed_mps,
            "max_observed_speed_mps": round(self.max_observed_speed_mps, 3),
            "min_raw_clearance_m": round(self.min_raw_clearance_m, 3),
            "candidate_min_raw_clearance_m": round(
                self.candidate_min_raw_clearance_m, 3
            ),
            "min_raw_clearance_event": self.min_raw_clearance_event,
            "status_reasons": sorted(self.status_reasons),
            "pilot_alerts": self.alerts,
            "setpoint_count": self.setpoint_count,
            "final_position_m": tuple(round(value, 2) for value in self.position()),
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", required=True, type=Path)
    parser.add_argument("--route", required=True, type=Path)
    parser.add_argument("--own-aircraft-index", required=True, type=int)
    parser.add_argument("--summary", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node: ReplayScenario | None = None
    try:
        node = ReplayScenario(args.capture, args.route, args.own_aircraft_index)
        result = node.run()
    except Exception as error:
        result = {
            "scenario": "replay",
            "result": "FAIL",
            "error": f"{type(error).__name__}: {error}",
        }
        if node is not None:
            result.update(
                status_reasons=sorted(node.status_reasons),
                min_raw_clearance_m=(
                    node.min_raw_clearance_m
                    if math.isfinite(node.min_raw_clearance_m)
                    else None
                ),
                pilot_alerts=node.alerts,
            )
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
