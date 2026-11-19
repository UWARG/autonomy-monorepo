"""Analyze a recorded CSV using independently measured trial ground truth."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def quaternion_angle(a, b):
    """Principal relative angle in degrees, invariant to quaternion sign."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(min(1.0, max(0.0, dot))))


def load_samples(path):
    samples = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), start=2):
            try:
                t = float(row["received_monotonic_s"])
                p = tuple(float(row[k]) for k in ("x_m", "y_m", "z_m"))
                q = tuple(float(row[k]) for k in ("qx", "qy", "qz", "qw"))
                if not all(math.isfinite(v) for v in (t, *p, *q)):
                    raise ValueError("non-finite value")
                norm = math.hypot(*q)
                if norm < 1e-12 or not math.isfinite(norm):
                    raise ValueError("invalid quaternion")
                if samples and t <= samples[-1][0]:
                    raise ValueError("receipt times must strictly increase")
                samples.append((t, p, tuple(v / norm for v in q)))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"Invalid CSV row {line}: {error}") from error
    if len(samples) < 2:
        raise ValueError("At least two valid pose samples are required")
    return samples


def endpoint_mean(samples):
    p = tuple(statistics.mean(s[1][axis] for s in samples) for axis in range(3))
    reference = samples[0][2]
    aligned = [
        tuple(-v for v in s[2])
        if sum(a * b for a, b in zip(reference, s[2])) < 0
        else s[2]
        for s in samples
    ]
    q = tuple(statistics.mean(s[axis] for s in aligned) for axis in range(4))
    norm = math.hypot(*q)
    if norm < 1e-12:
        raise ValueError("Endpoint orientation is ambiguous; hold still at endpoints")
    return p, tuple(v / norm for v in q)


def analyze(
    samples,
    *,
    trial,
    endpoint_window_s=1.0,
    expected_distance_m=None,
    expected_angle_deg=None,
    path_length_m=None,
    position_tolerance_m=None,
    angle_tolerance_deg=None,
    max_gap_s=None,
):
    if trial not in ("stationary", "translation", "rotation", "closed-loop"):
        raise ValueError("Unknown trial type")
    for name, value in locals().copy().items():
        if name in ("samples", "trial") or value is None:
            continue
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
    duration = samples[-1][0] - samples[0][0]
    if duration <= 0 or endpoint_window_s * 2 >= duration:
        raise ValueError("Recording must be longer than twice the endpoint window")
    if expected_angle_deg is not None and expected_angle_deg > 180:
        raise ValueError("Expected endpoint angle must be between 0 and 180 degrees")
    if path_length_m is not None and path_length_m <= 0:
        raise ValueError("Measured physical path length must be positive")
    if trial == "translation" and expected_distance_m is None:
        raise ValueError("Translation requires an independently measured distance")
    if trial == "rotation" and expected_angle_deg is None:
        raise ValueError("Rotation requires an independently measured endpoint angle")
    start = [s for s in samples if s[0] <= samples[0][0] + endpoint_window_s]
    end = [s for s in samples if s[0] >= samples[-1][0] - endpoint_window_s]
    p0, q0 = endpoint_mean(start)
    p1, q1 = endpoint_mean(end)
    displacement = math.dist(p0, p1)
    angle = quaternion_angle(q0, q1)
    maximum_distance = max(math.dist(p0, s[1]) for s in samples)
    metrics = {
        "samples": len(samples),
        "receipt_duration_s": duration,
        "endpoint_window_s": endpoint_window_s,
        "start_window_samples": len(start),
        "end_window_samples": len(end),
        "endpoint_displacement_m": displacement,
        "endpoint_rotation_deg": angle,
        "maximum_distance_from_start_m": maximum_distance,
        "maximum_rotation_from_start_deg": max(
            quaternion_angle(q0, s[2]) for s in samples
        ),
        "maximum_receipt_gap_s": max(b[0] - a[0] for a, b in zip(samples, samples[1:])),
    }
    position_error = (
        maximum_distance if trial in ("stationary", "rotation") else displacement
    )
    if trial == "translation":
        position_error = abs(displacement - expected_distance_m)
        metrics["translation_distance_error_m"] = position_error
    if trial == "closed-loop" and path_length_m is not None:
        metrics["endpoint_drift_percent_of_measured_path"] = (
            100 * displacement / path_length_m
        )
    if trial == "rotation":
        metrics["endpoint_angle_error_deg"] = abs(angle - expected_angle_deg)
    checks = {}
    if position_tolerance_m is not None:
        checks["position"] = position_error <= position_tolerance_m
    if trial == "rotation" and angle_tolerance_deg is not None:
        checks["angle"] = metrics["endpoint_angle_error_deg"] <= angle_tolerance_deg
    if max_gap_s is not None:
        checks["receipt_gap"] = metrics["maximum_receipt_gap_s"] <= max_gap_s
    required = {"position", "angle"} if trial == "rotation" else {"position"}
    assessment = "not_assessed"
    if required.issubset(checks):
        assessment = "pass" if all(checks.values()) else "fail"
    return {
        "trial": trial,
        "metrics": metrics,
        "checks": checks,
        "assessment": assessment,
        "criteria": {
            "expected_distance_m": expected_distance_m,
            "expected_angle_deg": expected_angle_deg,
            "path_length_m": path_length_m,
            "position_tolerance_m": position_tolerance_m,
            "angle_tolerance_deg": angle_tolerance_deg,
            "max_gap_s": max_gap_s,
        },
        "limitations": [
            "Hold camera stationary during both endpoint windows.",
            "Distance/angle magnitudes do not validate axis directions or body-frame transforms.",
            "Receipt gaps are not sensor latency or estimator frequency.",
            "Assessment covers supplied trial criteria only; it is not flight-readiness certification.",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument(
        "--trial",
        required=True,
        choices=["stationary", "translation", "rotation", "closed-loop"],
    )
    parser.add_argument("--output", type=Path, help="Optional new JSON report")
    parser.add_argument("--endpoint-window-s", type=float, default=1.0)
    for flag in (
        "expected-distance-m",
        "expected-angle-deg",
        "path-length-m",
        "position-tolerance-m",
        "angle-tolerance-deg",
        "max-gap-s",
    ):
        parser.add_argument("--" + flag, type=float)
    args = vars(parser.parse_args(argv))
    path = args.pop("input")
    output = args.pop("output")
    try:
        result = analyze(load_samples(path), **args)
        text = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if output:
            with output.open("x", encoding="utf-8") as stream:
                stream.write(text)
        print(text, end="")
        return 1 if result["assessment"] == "fail" else 0
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
