#!/usr/bin/env python3
"""Check SITL aircraft separation against the mock server's sampled true path."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path

EARTH_RADIUS_M = 6_371_000.0


def load_jsonl(path: Path) -> list[dict]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines(keepends=True)
        if line.endswith("\n")
    ]


def clearance_m(own: dict, before: dict, after: dict) -> float | None:
    start_s = before["monotonic_s"]
    end_s = after["monotonic_s"]
    if end_s - start_s > 0.25:
        return None
    fraction = (own["monotonic_s"] - start_s) / max(end_s - start_s, 1e-9)
    before_by_index = {item["aircraftIndex"]: item for item in before["aircraft"]}
    after_by_index = {item["aircraftIndex"]: item for item in after["aircraft"]}
    margins = []
    for index in before_by_index.keys() & after_by_index.keys():
        first = before_by_index[index]
        second = after_by_index[index]
        first_pos = first["position"]
        second_pos = second["position"]
        latitude = first_pos["lat"] + fraction * (second_pos["lat"] - first_pos["lat"])
        longitude = first_pos["lon"] + fraction * (second_pos["lon"] - first_pos["lon"])
        north_m = math.radians(latitude - own["latitude_deg"]) * EARTH_RADIUS_M
        east_m = (
            math.radians(longitude - own["longitude_deg"])
            * EARTH_RADIUS_M
            * math.cos(math.radians(own["latitude_deg"]))
        )
        margins.append(math.hypot(east_m, north_m) - first["horizontalKeepAway"])
    return min(margins) if margins else None


def evaluate(own_path: Path, truth_path: Path, minimum_margin_m: float) -> dict:
    own_samples = [
        row for row in load_jsonl(own_path)
        if row.get("armed") and row.get("mode") == "GUIDED"
        and row.get("latitude_deg") is not None
        and row.get("longitude_deg") is not None
    ]
    truth = load_jsonl(truth_path)
    times = [row["monotonic_s"] for row in truth]
    margins = []
    for own in own_samples:
        index = bisect.bisect_right(times, own["monotonic_s"])
        if index == 0 or index == len(truth):
            continue
        margin = clearance_m(own, truth[index - 1], truth[index])
        if margin is not None:
            margins.append(margin)
    coverage = len(margins) / len(own_samples) if own_samples else 0.0
    result = {
        "minimum_true_clearance_m": round(min(margins), 3) if margins else None,
        "required_clearance_m": minimum_margin_m,
        "samples": len(margins),
        "coverage": round(coverage, 3),
    }
    result["verdict"] = (
        "PASS" if margins and coverage >= 0.95
        and min(margins) >= minimum_margin_m else "FAIL"
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--own-jsonl", type=Path, required=True)
    parser.add_argument("--truth-jsonl", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--minimum-margin-m", type=float, default=3.0)
    args = parser.parse_args()
    result = evaluate(args.own_jsonl, args.truth_jsonl, args.minimum_margin_m)
    args.summary_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    if result["verdict"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
