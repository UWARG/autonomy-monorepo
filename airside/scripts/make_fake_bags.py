#!/usr/bin/env python3
"""
Create a fake airside bags directory for developing and testing pull_bags.py
without a drone.

The layout mirrors what engine.launch.py produces on the Pi:

    <out>/airside_<UTC time>/
        airside_<UTC time>_0.mcap
        airside_<UTC time>_1.mcap
        metadata.yaml            (only when the recorder exited cleanly)

The .mcap files are filler bytes behind the MCAP magic, so they are not
readable by ros2 bag. Only the directory layout, file sizes and metadata.yaml
(rosbag2 Humble, version 5) are realistic.
"""

import argparse
import os
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

_MCAP_MAGIC = b"\x89MCAP0\r\n"
_SPLIT_SECONDS = 300  # matches _BAG_SPLIT_SECONDS in engine.launch.py
_NS = 1_000_000_000

# A subset of _RECORDED_TOPICS with their message types and rough rates (Hz).
_TOPICS = [
    ("/mavros/state", "mavros_msgs/msg/State", 1),
    ("/mavros/global_position/global", "sensor_msgs/msg/NavSatFix", 10),
    ("/mavros/imu/data", "sensor_msgs/msg/Imu", 50),
    ("/heartbeat", "std_msgs/msg/Empty", 1),
    ("/rosout", "rcl_interfaces/msg/Log", 5),
]

# (start offset from base time, duration in seconds, kind)
# kind: "complete", "incomplete" (no metadata.yaml), "empty" (no files)
_SCENARIOS = [
    (timedelta(minutes=0), 760, "complete"),  # 12m40s flight, 3 splits
    (timedelta(minutes=30), 95, "complete"),  # short bench test, 1 split
    (timedelta(hours=2), 430, "incomplete"),  # power cut during second split
    (timedelta(hours=3), 0, "empty"),  # recorder died on startup
]


def bag_name(start):
    return start.strftime("airside_%Y-%m-%dT%H-%M-%SZ")


def write_mcap(path, size_bytes):
    """Write an MCAP-magic header followed by filler up to size_bytes."""
    with open(path, "wb") as f:
        f.write(_MCAP_MAGIC)
        f.write(os.urandom(max(size_bytes - len(_MCAP_MAGIC), 0)))


def topic_counts(duration_s):
    return [(name, type_, int(rate * duration_s)) for name, type_, rate in _TOPICS]


def metadata_yaml(name, start, duration_s, splits):
    start_ns = int(start.timestamp()) * _NS
    counts = topic_counts(duration_s)
    lines = [
        "rosbag2_bagfile_information:",
        "  version: 5",
        "  storage_identifier: mcap",
        "  duration:",
        f"    nanoseconds: {duration_s * _NS}",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start_ns}",
        f"  message_count: {sum(c for _, _, c in counts)}",
        "  topics_with_message_count:",
    ]
    for topic, type_, count in counts:
        lines += [
            "    - topic_metadata:",
            f"        name: {topic}",
            f"        type: {type_}",
            "        serialization_format: cdr",
            '        offered_qos_profiles: ""',
            f"      message_count: {count}",
        ]
    lines += [
        '  compression_format: ""',
        '  compression_mode: ""',
        "  relative_file_paths:",
    ]
    lines += [f"    - {name}_{i}.mcap" for i, _ in enumerate(splits)]
    lines.append("  files:")
    for i, split_s in enumerate(splits):
        split_start_ns = start_ns + i * _SPLIT_SECONDS * _NS
        lines += [
            f"    - path: {name}_{i}.mcap",
            "      starting_time:",
            f"        nanoseconds_since_epoch: {split_start_ns}",
            "      duration:",
            f"        nanoseconds: {split_s * _NS}",
            f"      message_count: {sum(c for _, _, c in topic_counts(split_s))}",
        ]
    return "\n".join(lines) + "\n"


def split_durations(duration_s):
    """Break a recording into 5-minute splits, last one partial."""
    splits = []
    remaining = duration_s
    while remaining > 0:
        splits.append(min(remaining, _SPLIT_SECONDS))
        remaining -= _SPLIT_SECONDS
    return splits


def make_bag(out, start, duration_s, kind, bytes_per_second):
    name = bag_name(start)
    bag_dir = out / name
    bag_dir.mkdir()
    if kind == "empty":
        os.utime(bag_dir, (start.timestamp(), start.timestamp()))
        return name

    splits = split_durations(duration_s)
    for i, split_s in enumerate(splits):
        path = bag_dir / f"{name}_{i}.mcap"
        write_mcap(path, split_s * bytes_per_second)
        # Backdate to when this split stopped being written, so tools that
        # use mtime to spot a live recording see these as finished.
        end = start.timestamp() + i * _SPLIT_SECONDS + split_s
        os.utime(path, (end, end))

    if kind == "complete":
        (bag_dir / "metadata.yaml").write_text(
            metadata_yaml(name, start, duration_s, splits)
        )
    return name


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", type=Path, help="directory to create, e.g. ./fake_bags")
    parser.add_argument(
        "--bytes-per-second",
        type=int,
        default=20_000,
        help="filler size per recorded second (default 20000, ~6 MB per 5 min split)",
    )
    args = parser.parse_args()

    # Never write into a non-empty directory: this could be run on the Pi,
    # where data/bags holds real flights.
    if args.out.exists() and (not args.out.is_dir() or any(args.out.iterdir())):
        sys.exit(f"{args.out} already exists and is not an empty directory")
    args.out.mkdir(parents=True, exist_ok=True)

    base = datetime(2026, 11, 8, 15, 2, 10, tzinfo=timezone.utc)
    for offset, duration_s, kind in _SCENARIOS:
        name = make_bag(args.out, base + offset, duration_s, kind, args.bytes_per_second)
        print(f"{kind:<10} {name}")

    # Something that is not a bag, which the helper must ignore.
    (args.out / "notes.txt").write_text("not a bag\n")
    print(f"{'stray':<10} notes.txt")


if __name__ == "__main__":
    main()
