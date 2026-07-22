"""Derive follow timing and enforce the non-negotiable Gate-5 thresholds."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import Counter

REQUIRED_SCENARIOS = {
    "static",
    "lateral",
    "approach_recede",
    "crossing",
    "occlusion_0.5s",
    "occlusion_1s",
    "occlusion_2s",
}


def percentile(values, q):
    values = sorted(values)
    if not values:
        return math.nan
    position = (len(values) - 1) * q
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def choose_alpha(period_s):
    for tenth in range(1, 11):
        alpha = tenth / 10.0
        if ((1.0 - alpha) / alpha) * period_s <= 0.05 + 1e-12:
            return alpha
    return 1.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument(
        "--soak-results",
        help="JSON with passing_vmax_mps values from deterministic replay/500-episode soak",
    )
    parser.add_argument("--json-out")
    args = parser.parse_args()
    rows = list(csv.DictReader(open(args.csv_path, newline="", encoding="utf-8")))
    targets = [row for row in rows if row["event"] == "target"]
    captures = [float(row["capture_time_s"]) for row in targets]
    latencies = [
        float(row["ros_receive_s"]) - float(row["capture_time_s"])
        for row in targets
    ]
    periods = [b - a for a, b in zip(captures, captures[1:]) if b > a]
    host_times = [float(row["host_time_s"]) for row in rows if row["host_time_s"]]
    duration = max(host_times) - min(host_times) if len(host_times) >= 2 else 0.0
    session_start = min(host_times) if host_times else 0.0
    bins = Counter(math.floor(value - session_start) for value in captures)
    bin_count = math.floor(duration)
    fps_samples = [bins.get(index, 0) for index in range(bin_count)]
    p05_fps = percentile(fps_samples, 0.05)
    mean_period = statistics.mean(periods) if periods else math.inf
    source_fps = 1.0 / mean_period if math.isfinite(mean_period) else 0.0
    stream_hz = min(50.0, max(20.0, math.ceil(source_fps / 5.0) * 5.0))
    p99_latency = percentile(latencies, 0.99)
    mean_latency = statistics.mean(latencies) if latencies else math.nan
    latency_jitter_p99 = percentile(
        [abs(value - mean_latency) for value in latencies], 0.99
    )
    jitter_p99 = percentile(
        [abs(period - mean_period) for period in periods], 0.99
    )

    zero_delays = []
    previous_setpoint_was_zero = True
    for row in rows:
        if row["event"] != "setpoint" or not row["latest_capture_s"]:
            continue
        velocity = [float(row[key]) for key in ("sp_vx", "sp_vy", "sp_vz")]
        is_zero = max(abs(value) for value in velocity) <= 1e-6
        if is_zero and not previous_setpoint_was_zero:
            zero_delays.append(float(row["host_time_s"]) - float(row["latest_capture_s"]))
        previous_setpoint_was_zero = is_zero
    p99_zero = percentile(zero_delays, 0.99)
    reaction_time = p99_zero + 1.0 / stream_hz + 0.05
    freshness = p99_latency + 2.0 * mean_period
    scenarios = {row["scenario"] for row in rows if row["scenario"]}
    gap_sizes = [int(row["sequence_gap"] or 0) for row in targets]
    dropped = sum(gap_sizes)
    dropout_rate = dropped / (len(targets) + dropped) if targets else 1.0

    passing_vmax = []
    if args.soak_results:
        payload = json.load(open(args.soak_results, encoding="utf-8"))
        passing_vmax = [float(value) for value in payload.get("passing_vmax_mps", [])]
    selected_vmax = max((value for value in passing_vmax if value <= 1.5), default=None)

    checks = {
        "duration_at_least_5_minutes": duration >= 300.0,
        "all_scenarios_present": REQUIRED_SCENARIOS <= scenarios,
        "capture_to_zero_transition_recorded": bool(zero_delays),
        "p05_fps_at_least_10": p05_fps >= 10.0,
        "capture_to_receive_p99_at_most_300ms": p99_latency <= 0.300,
        "passing_vmax_at_least_0.5": selected_vmax is not None and selected_vmax >= 0.5,
    }
    result = {
        "gate_5_pass": all(checks.values()),
        "checks": checks,
        "samples": len(targets),
        "duration_s": duration,
        "p05_fps": p05_fps,
        "source_fps": source_fps,
        "capture_to_receive_p99_s": p99_latency,
        "capture_period_jitter_p99_s": jitter_p99,
        "latency_jitter_p99_s": latency_jitter_p99,
        "sequence_gaps": dropped,
        "dropout_rate": dropout_rate,
        "dropout_gap_sizes": [value for value in gap_sizes if value],
        "capture_to_zero_p99_s": p99_zero,
        "capture_to_zero_samples": len(zero_delays),
        "reaction_time_s": reaction_time,
        "target_freshness_s": freshness,
        "stream_hz": stream_hz,
        "ema_alpha": choose_alpha(mean_period),
        "selected_vmax_mps": selected_vmax,
        "scenarios": sorted(scenarios),
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2, sort_keys=True)
            output.write("\n")
    sys.exit(0 if result["gate_5_pass"] else 1)


if __name__ == "__main__":
    main()
