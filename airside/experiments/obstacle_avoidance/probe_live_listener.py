#!/usr/bin/env python3
"""Receive-only check of the real listener's ROS traffic snapshots."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from airside_interfaces.msg import TrafficSnapshot
from comms.traffic_listener_node import TrafficListener
from rclpy.executors import SingleThreadedExecutor


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=15.0)
    args = parser.parse_args()
    if not 0.0 < args.duration_s <= 120.0:
        parser.error("duration must be between 0 and 120 seconds")

    rclpy.init()
    listener = TrafficListener()
    observer = rclpy.create_node("traffic_probe")
    count = 0
    healthy = 0
    indices: set[int] = set()

    def receive(snapshot: TrafficSnapshot) -> None:
        nonlocal count, healthy
        count += 1
        if snapshot.connected and snapshot.healthy:
            healthy += 1
            indices.update(aircraft.aircraft_index for aircraft in snapshot.aircraft)

    observer.create_subscription(TrafficSnapshot, "/aeac/traffic", receive, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(observer)
    initial_sequence = listener._sequence
    try:
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.2, deadline - time.monotonic()))
    finally:
        executor.remove_node(observer)
        executor.remove_node(listener)
        observer.destroy_node()
        listener.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

    print(
        json.dumps(
            {
                "ros_snapshots": count,
                "healthy_snapshots": healthy,
                "aircraft_indices": sorted(indices),
                "listener_deliveries": listener._sequence - initial_sequence,
            },
            sort_keys=True,
        )
    )
    return 0 if healthy else 2


if __name__ == "__main__":
    raise SystemExit(main())
