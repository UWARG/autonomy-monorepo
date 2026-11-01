#!/usr/bin/env python3
"""Summarize sender cadence and MAVROS input gaps from a recorded SITL bag."""

from __future__ import annotations

import argparse
import collections
import json
import sqlite3
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix


def analyze(database: Path) -> dict:
    with sqlite3.connect(database) as connection:
        topics = dict(connection.execute("SELECT name, id FROM topics"))
        diagnostics = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/aeac/telemetry_diagnostics"],),
        ).fetchall()
        gps_rows = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/mavros/global_position/global"],),
        ).fetchall()
        gps_timestamps = [row[0] / 1e9 for row in gps_rows]
        state_timestamps = [row[0] / 1e9 for row in connection.execute(
            "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/mavros/state"],),
        )]
        traffic_timestamps = [row[0] / 1e9 for row in connection.execute(
            "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/aeac/live_traffic"],),
        )]
        planner_rows = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/obstacle_avoidance/diagnostics"],),
        ).fetchall()
    reasons: collections.Counter[str] = collections.Counter()
    late_sends: list[dict] = []
    stale_input_events: list[float] = []
    diagnostic_gaps: list[dict] = []
    previous_diagnostic_s: float | None = None
    previous_send_s: float | None = None
    previous_send_count = 0
    for timestamp, data in diagnostics:
        diagnostic_s = timestamp / 1e9
        message = deserialize_message(data, DiagnosticArray)
        if not message.status:
            continue
        status = message.status[0]
        if previous_diagnostic_s is not None and diagnostic_s - previous_diagnostic_s > 1.1:
            diagnostic_gaps.append({"unix_s": diagnostic_s,
                                    "gap_s": round(diagnostic_s - previous_diagnostic_s, 3),
                                    "reason": status.message})
        previous_diagnostic_s = diagnostic_s
        reasons[status.message] += 1
        if status.message == "STALE_TELEMETRY_INPUT":
            stale_input_events.append(diagnostic_s)
        fields = {item.key: item.value for item in status.values}
        interval_text = fields.get("last_send_interval_s", "")
        send_count = int(fields.get("sent_count", "0"))
        if interval_text and float(interval_text) > 1.1:
            late_sends.append({"unix_s": timestamp / 1e9,
                               "interval_s": float(interval_text),
                               "bag_wall_interval_s": (
                                   round(diagnostic_s - previous_send_s, 3)
                                   if previous_send_s is not None else None
                               ),
                               "reason": status.message})
        if send_count > previous_send_count:
            previous_send_s = diagnostic_s
            previous_send_count = send_count

    def maximum_gap(timestamps: list[float]) -> float | None:
        return max(
            (later - earlier for earlier, later in zip(timestamps, timestamps[1:])),
            default=None,
        )

    def long_gaps(timestamps: list[float], threshold_s: float = 1.1) -> list[dict]:
        return [
            {"unix_s": later, "gap_s": round(later - earlier, 3)}
            for earlier, later in zip(timestamps, timestamps[1:])
            if later - earlier > threshold_s
        ]

    gps_stamp_ages = []
    for timestamp, data in gps_rows:
        fix = deserialize_message(data, NavSatFix)
        stamp_s = fix.header.stamp.sec + fix.header.stamp.nanosec / 1e9
        gps_stamp_ages.append((timestamp / 1e9, timestamp / 1e9 - stamp_s))
    planner_holds = []
    previous_hold_count = 0
    for timestamp, data in planner_rows:
        message = deserialize_message(data, DiagnosticArray)
        if not message.status:
            continue
        status = message.status[0]
        fields = {item.key: item.value for item in status.values}
        hold_count = int(fields.get("hold_count", "0"))
        if hold_count > previous_hold_count:
            planner_holds.append({"unix_s": timestamp / 1e9,
                                  "reason": fields.get("reason"),
                                  "source_age_s": fields.get("source_age_s")})
        previous_hold_count = hold_count

    return {
        "telemetry_diagnostic_reasons": dict(reasons),
        "late_sends": late_sends,
        "stale_input_unix_s": stale_input_events,
        "long_diagnostic_gaps": diagnostic_gaps,
        "gps_message_count": len(gps_timestamps),
        "maximum_gps_reception_gap_s": maximum_gap(gps_timestamps),
        "maximum_gps_stamp_age_s": max((age for _, age in gps_stamp_ages), default=None),
        "gps_stamp_age_over_2_5_s": [
            {"unix_s": instant, "age_s": round(age, 3)}
            for instant, age in gps_stamp_ages if age > 2.5
        ][:20],
        "long_gps_reception_gaps": long_gaps(gps_timestamps),
        "maximum_state_reception_gap_s": maximum_gap(state_timestamps),
        "long_state_reception_gaps": long_gaps(state_timestamps),
        "long_traffic_reception_gaps": long_gaps(traffic_timestamps, 1.5),
        "planner_holds": planner_holds,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("database", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.database), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
