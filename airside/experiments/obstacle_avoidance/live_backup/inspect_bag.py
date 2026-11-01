#!/usr/bin/env python3
"""Summarize FCU update cadence and controller diagnostics from a SITL rosbag."""

from __future__ import annotations

import argparse
import collections
import math
import statistics

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import NavSatFix


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()
    print("bag:", args.bag)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="sqlite3"), ConverterOptions("", ""))
    events: collections.Counter[str] = collections.Counter()
    gps_stamp_ages: list[float] = []
    gps_times: list[float] = []
    state_times: list[float] = []
    stale_events: list[tuple[float, str]] = []
    speeds_mps: list[float] = []
    previous_pose: tuple[float, float, float] | None = None
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic == "/mavros/global_position/global":
            fix = deserialize_message(raw, NavSatFix)
            stamp_s = fix.header.stamp.sec + fix.header.stamp.nanosec / 1e9
            gps_stamp_ages.append(timestamp_ns / 1e9 - stamp_s)
            gps_times.append(timestamp_ns / 1e9)
        elif topic == "/mavros/state":
            state_times.append(timestamp_ns / 1e9)
        elif topic == "/mavros/local_position/pose":
            pose = deserialize_message(raw, PoseStamped)
            current = (
                pose.pose.position.x,
                pose.pose.position.y,
                timestamp_ns / 1e9,
            )
            if previous_pose is not None:
                dt = current[2] - previous_pose[2]
                if dt >= 0.08:
                    speeds_mps.append(
                        math.hypot(
                            current[0] - previous_pose[0],
                            current[1] - previous_pose[1],
                        ) / dt
                    )
            previous_pose = current
        elif topic == "/position_controller/diagnostics":
            diagnostic = deserialize_message(raw, DiagnosticArray)
            for status in diagnostic.status:
                events[status.message] += 1
                if status.message.startswith("STALE"):
                    stale_events.append((timestamp_ns / 1e9, status.message))
    print("diagnostics:", dict(events))
    if gps_stamp_ages:
        print(
            "GPS stamp age (s):",
            {"min": round(min(gps_stamp_ages), 3),
             "median": round(statistics.median(gps_stamp_ages), 3),
             "max": round(max(gps_stamp_ages), 3)},
        )
    for label, times in (("GPS", gps_times), ("FCU state", state_times)):
        if len(times) > 1:
            gaps = [(round(b - a, 3), round(b, 3)) for a, b in zip(times, times[1:])]
            print(f"{label} largest gaps (s, arrival):", sorted(gaps, reverse=True)[:5])
    if speeds_mps:
        ordered = sorted(speeds_mps)
        print(
            "horizontal speed (m/s):",
            {
                "samples": len(ordered),
                "median": round(statistics.median(ordered), 3),
                "p95": round(ordered[int(0.95 * (len(ordered) - 1))], 3),
                "max": round(ordered[-1], 3),
                "above_1_mps": sum(speed > 1.0 for speed in ordered),
            },
        )
    print("stale events:", [(round(at, 3), reason) for at, reason in stale_events])


if __name__ == "__main__":
    main()
