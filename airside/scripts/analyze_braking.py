"""Measure real BRAKE deceleration from a Gate-6 recorder CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--json-out")
    parser.add_argument("--stop-speed", type=float, default=0.1)
    args = parser.parse_args()
    with open(args.csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    brake_time = next(
        (
            float(row["host_time_s"])
            for row in rows
            if row["event"] == "mode_transition" and row["mode"] == "BRAKE"
        ),
        None,
    )
    samples = []
    if brake_time is not None:
        for row in rows:
            if row["event"] != "vehicle" or float(row["host_time_s"]) < brake_time:
                continue
            if not row["vehicle_vx"]:
                continue
            speed = math.hypot(float(row["vehicle_vx"]), float(row["vehicle_vy"]))
            samples.append(
                (
                    float(row["host_time_s"]),
                    float(row["vehicle_x"]),
                    float(row["vehicle_y"]),
                    speed,
                )
            )

    stopped_index = next(
        (index for index, sample in enumerate(samples) if sample[3] <= args.stop_speed),
        None,
    )
    passed = brake_time is not None and len(samples) >= 3 and stopped_index is not None
    result = {"gate_6_measurement_valid": passed, "brake_mode_time_s": brake_time}
    if passed:
        used = samples[: stopped_index + 1]
        initial_speed = used[0][3]
        stop_time = used[-1][0] - brake_time
        distance = sum(
            math.hypot(b[1] - a[1], b[2] - a[2])
            for a, b in zip(used, used[1:])
        )
        deceleration = initial_speed**2 / (2.0 * distance) if distance > 0.0 else math.inf
        result.update(
            initial_speed_mps=initial_speed,
            stop_time_s=stop_time,
            stop_distance_m=distance,
            equivalent_constant_deceleration_mps2=deceleration,
            sample_count=len(used),
        )
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2, sort_keys=True)
            handle.write("\n")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
