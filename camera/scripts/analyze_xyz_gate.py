"""Enforce the physical XYZ calibration matrix required before flight."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys

AXES = ("x", "y", "z")
DEPTHS_MM = (1000.0, 1500.0, 2000.0, 2500.0)
OFFSETS_MM = ((0.0, 0.0), (300.0, 0.0), (-300.0, 0.0), (0.0, 300.0), (0.0, -300.0))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--json-out")
    args = parser.parse_args()
    with open(args.csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    groups = {}
    for row in rows:
        pose = tuple(float(row[f"gt_{axis}_mm"]) for axis in AXES)
        groups.setdefault(pose, []).append(row)

    required = {
        (x, y, z)
        for z in DEPTHS_MM
        for x, y in OFFSETS_MM
    }
    results = []
    gate_pass = True
    for pose in sorted(required, key=lambda value: (value[2], value[0], value[1])):
        samples = groups.get(pose, [])
        pose_result = {"pose_mm": pose, "samples": len(samples), "axes": {}}
        if len(samples) < 100:
            gate_pass = False
        for index, axis in enumerate(AXES):
            truth = pose[index]
            range_mm = pose[2]
            calibrated = [float(row[f"cal_{axis}_mm"]) for row in samples]
            raw = [float(row[f"raw_{axis}_mm"]) for row in samples]
            if calibrated:
                cal_error_pct = abs(statistics.median(calibrated) - truth) / range_mm * 100.0
                raw_error_pct = abs(statistics.median(raw) - truth) / range_mm * 100.0
                std_mm = statistics.pstdev(calibrated)
            else:
                cal_error_pct = raw_error_pct = std_mm = float("inf")
            passed = (
                len(samples) >= 100
                and cal_error_pct <= 10.0
                and std_mm <= 50.0
                and cal_error_pct <= raw_error_pct + 2.0
            )
            gate_pass &= passed
            pose_result["axes"][axis] = {
                "calibrated_median_absolute_error_pct_of_range": cal_error_pct,
                "raw_median_absolute_error_pct_of_range": raw_error_pct,
                "calibrated_std_mm": std_mm,
                "pass": passed,
            }
        results.append(pose_result)

    extra = sorted(set(groups) - required)
    report = {
        "calibration_gate_pass": gate_pass,
        "required_pose_count": len(required),
        "captured_required_pose_count": len(required & set(groups)),
        "extra_poses_mm": extra,
        "poses": results,
    }
    print(json.dumps(report, indent=2))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")
    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
