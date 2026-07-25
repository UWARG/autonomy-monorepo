"""Derive detector-aware follow timing and enforce Gate-5 thresholds."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import Counter, defaultdict

REQUIRED_SCENARIOS = {
    "static",
    "lateral",
    "approach_recede",
    "crossing",
    "occlusion_0.5s",
    "occlusion_1s",
    "occlusion_2s",
    "proximity_stop",
}


def percentile(values, quantile):
    values = sorted(values)
    if not values:
        return math.inf
    position = (len(values) - 1) * quantile
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


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _float(row: dict[str, str], key: str, fallback: str | None = None):
    value = row.get(key, "")
    if not value and fallback is not None:
        value = row.get(fallback, "")
    return float(value) if value not in ("", None) else None


def _fps_samples(captures: list[float], start: float, duration: float):
    bins = Counter(math.floor(value - start) for value in captures)
    return [bins.get(index, 0) for index in range(max(0, math.floor(duration)))]


def _sequence_dropouts(sequences: list[int]):
    deltas = [
        later - earlier
        for earlier, later in zip(sequences, sequences[1:])
        if later > earlier
    ]
    expected_step = min(deltas) if deltas else 1
    gap_sizes = [max(0, math.ceil(delta / expected_step) - 1) for delta in deltas]
    return expected_step, gap_sizes


def analyze(rows: list[dict[str, str]], passing_vmax: list[float]) -> dict:
    targets = [row for row in rows if row.get("event") == "target"]
    detector_targets = [
        row for row in targets if _truthy(row.get("detector_confirmed", ""))
    ]
    tracker_captures = [
        value
        for row in targets
        if (value := _float(row, "tracker_capture_time_s", "capture_time_s"))
        is not None
    ]
    detector_captures = [
        value
        for row in detector_targets
        if (value := _float(row, "detector_capture_time_s", "capture_time_s"))
        is not None
    ]
    detector_latencies = [
        value
        for row in detector_targets
        if (value := _float(row, "detector_to_ros_latency_s")) is not None
    ]
    if not detector_latencies:
        detector_latencies = [
            float(row["ros_receive_s"])
            - float(row.get("detector_capture_time_s") or row["capture_time_s"])
            for row in detector_targets
        ]
    inference_ages = [
        value
        for row in targets
        if (value := _float(row, "inference_age_s")) is not None
    ]
    detector_periods = [
        later - earlier
        for earlier, later in zip(detector_captures, detector_captures[1:])
        if later > earlier
    ]
    tracker_periods = [
        later - earlier
        for earlier, later in zip(tracker_captures, tracker_captures[1:])
        if later > earlier
    ]
    host_times = [float(row["host_time_s"]) for row in rows if row.get("host_time_s")]
    duration = max(host_times) - min(host_times) if len(host_times) >= 2 else 0.0
    session_start = min(host_times) if host_times else 0.0
    detector_fps_samples = _fps_samples(detector_captures, session_start, duration)
    tracker_fps_samples = _fps_samples(tracker_captures, session_start, duration)
    detector_p05_fps = percentile(detector_fps_samples, 0.05)
    tracker_p05_fps = percentile(tracker_fps_samples, 0.05)
    mean_detector_period = (
        statistics.mean(detector_periods) if detector_periods else math.inf
    )
    detector_fps = (
        1.0 / mean_detector_period
        if math.isfinite(mean_detector_period) and mean_detector_period > 0.0
        else 0.0
    )
    mean_tracker_period = (
        statistics.mean(tracker_periods) if tracker_periods else math.inf
    )
    tracker_fps = (
        1.0 / mean_tracker_period
        if math.isfinite(mean_tracker_period) and mean_tracker_period > 0.0
        else 0.0
    )
    stream_hz = min(50.0, max(20.0, math.ceil(detector_fps / 5.0) * 5.0))
    p99_latency = percentile(detector_latencies, 0.99)
    mean_latency = (
        statistics.mean(detector_latencies) if detector_latencies else math.inf
    )
    latency_jitter_p99 = percentile(
        [abs(value - mean_latency) for value in detector_latencies], 0.99
    )
    detector_jitter_p99 = percentile(
        [abs(period - mean_detector_period) for period in detector_periods],
        0.99,
    )

    zero_transitions = []
    previous_setpoint_was_zero = True
    for row in rows:
        if row.get("event") != "setpoint":
            continue
        velocity = [float(row[key]) for key in ("sp_vx", "sp_vy", "sp_vz")]
        is_zero = max(abs(value) for value in velocity) <= 1e-6
        latest_detector = _float(row, "latest_detector_capture_s", "latest_capture_s")
        if is_zero and not previous_setpoint_was_zero and latest_detector is not None:
            zero_transitions.append(
                (
                    float(row["host_time_s"]),
                    latest_detector,
                    row.get("stop_reason", "") or "unspecified",
                    row.get("scenario", ""),
                )
            )
        previous_setpoint_was_zero = is_zero
    diagnostic_reasons = [
        (float(row["host_time_s"]), row.get("stop_reason", ""))
        for row in rows
        if row.get("event") == "diagnostic" and row.get("host_time_s")
    ]
    zero_delays_by_reason = defaultdict(list)
    for zero_time, latest_detector, recorded_reason, scenario in zero_transitions:
        reason = recorded_reason
        if reason in ("", "none", "unspecified"):
            nearby = [
                (abs(diagnostic_time - zero_time), diagnostic_reason)
                for diagnostic_time, diagnostic_reason in diagnostic_reasons
                if abs(diagnostic_time - zero_time) <= 1.0
                and diagnostic_reason not in ("", "none")
            ]
            if nearby:
                reason = min(nearby)[1]
        if scenario == "proximity_stop" and reason == "proximity_emergency":
            zero_delays_by_reason[reason].append(zero_time - latest_detector)
        elif reason not in ("", "none", "unspecified"):
            zero_delays_by_reason[reason].append(zero_time - latest_detector)
    proximity_zero_delays = zero_delays_by_reason.get("proximity_emergency", [])
    p99_proximity_zero = percentile(proximity_zero_delays, 0.99)
    reaction_time = (
        p99_proximity_zero + 1.0 / stream_hz + 0.05
        if proximity_zero_delays
        else math.inf
    )
    freshness = (
        p99_latency + 2.0 * mean_detector_period
        if detector_latencies and detector_periods
        else math.inf
    )
    scenarios = {row["scenario"] for row in rows if row.get("scenario")}

    tracker_sequences = [int(row["sequence_num"]) for row in targets]
    detector_sequences = [int(row["detector_sequence_num"]) for row in detector_targets]
    _, tracker_gap_sizes = _sequence_dropouts(tracker_sequences)
    detector_stride, detector_gap_sizes = _sequence_dropouts(detector_sequences)
    detector_dropped = sum(detector_gap_sizes)
    detector_dropout_rate = (
        detector_dropped / (len(detector_targets) + detector_dropped)
        if detector_targets
        else 1.0
    )
    crossing_ids = {
        int(row["track_id"])
        for row in targets
        if row.get("scenario") == "crossing" and row.get("track_id")
    }
    crossing_identity_retained = bool(crossing_ids) and len(crossing_ids) == 1

    selected_vmax = max(
        (value for value in passing_vmax if 0.5 <= value <= 1.5),
        default=None,
    )
    checks = {
        "duration_at_least_5_minutes": duration >= 300.0,
        "all_scenarios_present": REQUIRED_SCENARIOS <= scenarios,
        "proximity_capture_to_zero_recorded": bool(proximity_zero_delays),
        "detector_p05_fps_at_least_10": detector_p05_fps >= 10.0,
        "detector_capture_to_receive_p99_at_most_300ms": p99_latency <= 0.300,
        "crossing_identity_retained": crossing_identity_retained,
        "passing_vmax_at_least_0.5": (
            selected_vmax is not None and selected_vmax >= 0.5
        ),
    }
    return {
        "gate_5_pass": all(checks.values()),
        "checks": checks,
        "detector_samples": len(detector_targets),
        "tracker_samples": len(targets),
        "tracker_only_samples": len(targets) - len(detector_targets),
        "duration_s": duration,
        # Compatibility aliases intentionally mean detector-confirmed timing.
        "p05_fps": detector_p05_fps,
        "source_fps": detector_fps,
        "capture_to_receive_p99_s": p99_latency,
        "detector_p05_fps": detector_p05_fps,
        "tracker_p05_fps": tracker_p05_fps,
        "detector_fps": detector_fps,
        "tracker_fps": tracker_fps,
        "detector_stride_frames": detector_stride,
        "detector_capture_to_ros_p99_s": p99_latency,
        "detector_capture_period_jitter_p99_s": detector_jitter_p99,
        "capture_period_jitter_p99_s": detector_jitter_p99,
        "latency_jitter_p99_s": latency_jitter_p99,
        "inference_age_p99_s": percentile(inference_ages, 0.99),
        "tracker_sequence_gaps": sum(tracker_gap_sizes),
        "detector_sequence_gaps": detector_dropped,
        "sequence_gaps": detector_dropped,
        "dropout_rate": detector_dropout_rate,
        "dropout_gap_sizes": [value for value in detector_gap_sizes if value],
        "reason_specific_capture_to_zero_p99_s": {
            reason: percentile(values, 0.99)
            for reason, values in zero_delays_by_reason.items()
        },
        "proximity_capture_to_zero_p99_s": p99_proximity_zero,
        "proximity_capture_to_zero_samples": len(proximity_zero_delays),
        "reaction_time_s": reaction_time,
        "target_freshness_s": freshness,
        "stream_hz": stream_hz,
        "ema_alpha": choose_alpha(mean_detector_period),
        "selected_vmax_mps": selected_vmax,
        "crossing_track_ids": sorted(crossing_ids),
        "scenarios": sorted(scenarios),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument(
        "--soak-results",
        help=(
            "JSON with passing_vmax_mps values from deterministic replay/"
            "500-episode soak"
        ),
    )
    parser.add_argument("--json-out")
    args = parser.parse_args()
    with open(args.csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    passing_vmax = []
    if args.soak_results:
        with open(args.soak_results, encoding="utf-8") as handle:
            payload = json.load(handle)
        passing_vmax = [float(value) for value in payload.get("passing_vmax_mps", [])]
    result = analyze(rows, passing_vmax)
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2, sort_keys=True)
            output.write("\n")
    sys.exit(0 if result["gate_5_pass"] else 1)


if __name__ == "__main__":
    main()
