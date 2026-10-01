#!/usr/bin/env python3
"""Wait for one healthy AEAC traffic snapshot without using the ROS CLI daemon."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from airside_interfaces.msg import TrafficSnapshot
from rclpy.node import Node


class TrafficProbe(Node):
    def __init__(self) -> None:
        super().__init__("traffic_snapshot_probe")
        self.snapshot: TrafficSnapshot | None = None
        self.subscription = self.create_subscription(
            TrafficSnapshot,
            "/aeac/traffic",
            self._callback,
            10,
        )

    def _callback(self, message: TrafficSnapshot) -> None:
        indices = {aircraft.aircraft_index for aircraft in message.aircraft}
        if message.connected and message.healthy and {1, 2}.issubset(indices):
            self.snapshot = message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = TrafficProbe()
    deadline_s = time.monotonic() + args.timeout
    try:
        while node.snapshot is None and time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.snapshot is None:
            return 1
        output = {
            "sequence": node.snapshot.sequence,
            "connected": node.snapshot.connected,
            "healthy": node.snapshot.healthy,
            "reason": node.snapshot.reason,
            "own_aircraft_index": node.snapshot.own_aircraft_index,
            "aircraft_indices": [
                aircraft.aircraft_index for aircraft in node.snapshot.aircraft
            ],
        }
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(output, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(output, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
