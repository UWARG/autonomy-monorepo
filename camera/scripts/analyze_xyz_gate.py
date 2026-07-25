"""Analyze independent fit/validation XYZ sessions and enforce the 3 m gate."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
from typing import Callable

import numpy as np

AXES = ("x", "y", "z")
DEPTHS_MM = (1000.0, 1500.0, 2000.0, 2500.0, 3000.0)
OFFSETS_MM = (
    (0.0, 0.0),
    (300.0, 0.0),
    (-300.0, 0.0),
    (0.0, 300.0),
    (0.0, -300.0),
)
MIN_SAMPLES_PER_POSE = 100


def required_poses() -> set[tuple[float, float, float]]:
    return {(x, y, z) for z in DEPTHS_MM for x, y in OFFSETS_MM}


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _pose(row: dict[str, str]) -> tuple[float, float, float]:
    return tuple(float(row[f"gt_{axis}_mm"]) for axis in AXES)


def _group(rows: list[dict[str, str]]):
    groups: dict[tuple[float, float, float], list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(_pose(row), []).append(row)
    return groups


def _axis_depth_features(row: dict[str, str], axis: str) -> list[float]:
    raw_axis = float(row[f"raw_{axis}_mm"])
    raw_depth_m = float(row["raw_z_mm"]) / 1000.0
    return [raw_axis, raw_axis * raw_depth_m, 1.0]


def fit_axis_depth_model(rows: list[dict[str, str]]) -> dict[str, list[float]]:
    """Fit ``truth = raw*(a + b*raw_z_m) + c`` independently per axis."""
    parameters = {}
    for axis in AXES:
        design = np.asarray(
            [_axis_depth_features(row, axis) for row in rows], dtype=float
        )
        truth = np.asarray([float(row[f"gt_{axis}_mm"]) for row in rows])
        coefficients, _, _, _ = np.linalg.lstsq(design, truth, rcond=None)
        parameters[axis] = [float(value) for value in coefficients]
    return parameters


def _predictor(
    model_name: str,
    parameters: dict[str, list[float]] | None = None,
) -> Callable[[dict[str, str], str], float]:
    if model_name == "raw":
        return lambda row, axis: float(row[f"raw_{axis}_mm"])
    if model_name == "shared_depth_ratio":
        return lambda row, axis: float(row[f"cal_{axis}_mm"])
    if model_name == "axis_depth_affine" and parameters is not None:
        return lambda row, axis: float(
            np.dot(_axis_depth_features(row, axis), parameters[axis])
        )
    raise ValueError(f"unknown or incomplete calibration model: {model_name}")


def _axis_stats(
    rows: list[dict[str, str]],
    axis: str,
    truth: float,
    range_mm: float,
    predict: Callable[[dict[str, str], str], float],
) -> dict:
    estimates = [predict(row, axis) for row in rows]
    raw = [float(row[f"raw_{axis}_mm"]) for row in rows]
    errors = [estimate - truth for estimate in estimates]
    raw_errors = [estimate - truth for estimate in raw]
    absolute = [abs(value) for value in errors]
    raw_absolute = [abs(value) for value in raw_errors]
    median_absolute_error_mm = statistics.median(absolute) if absolute else math.inf
    raw_median_absolute_error_mm = (
        statistics.median(raw_absolute) if raw_absolute else math.inf
    )
    error_pct = median_absolute_error_mm / range_mm * 100.0
    raw_error_pct = raw_median_absolute_error_mm / range_mm * 100.0
    std_mm = statistics.pstdev(errors) if errors else math.inf
    paired_delta = (
        statistics.median(
            model_error - raw_error
            for model_error, raw_error in zip(absolute, raw_absolute)
        )
        if errors
        else math.inf
    )
    passed = (
        len(rows) >= MIN_SAMPLES_PER_POSE
        and error_pct <= 10.0
        and std_mm <= 50.0
        and error_pct <= raw_error_pct + 2.0
    )
    return {
        "median_absolute_error_mm": median_absolute_error_mm,
        "median_absolute_error_pct_of_range": error_pct,
        "bias_mm": statistics.mean(errors) if errors else math.inf,
        "std_mm": std_mm,
        "p95_absolute_error_mm": percentile(absolute, 0.95),
        "raw_median_absolute_error_mm": raw_median_absolute_error_mm,
        "raw_median_absolute_error_pct_of_range": raw_error_pct,
        "paired_absolute_error_delta_mm": paired_delta,
        "pass": passed,
    }


def evaluate_model(
    rows: list[dict[str, str]],
    model_name: str,
    parameters: dict[str, list[float]] | None = None,
) -> dict:
    groups = _group(rows)
    required = required_poses()
    predict = _predictor(model_name, parameters)
    pose_results = []
    gate_pass = True
    worst_case_error_pct = 0.0
    for pose in sorted(required, key=lambda value: (value[2], value[0], value[1])):
        samples = groups.get(pose, [])
        result = {"pose_mm": pose, "samples": len(samples), "axes": {}}
        if len(samples) < MIN_SAMPLES_PER_POSE:
            gate_pass = False
        for index, axis in enumerate(AXES):
            stats = _axis_stats(samples, axis, pose[index], pose[2], predict)
            result["axes"][axis] = stats
            gate_pass &= stats["pass"]
            worst_case_error_pct = max(
                worst_case_error_pct,
                stats["median_absolute_error_pct_of_range"],
            )
        pose_results.append(result)
    return {
        "model": model_name,
        "pass": gate_pass,
        "worst_case_median_absolute_error_pct_of_range": worst_case_error_pct,
        "captured_required_pose_count": len(required & set(groups)),
        "missing_poses_mm": sorted(
            required - set(groups), key=lambda value: (value[2], value[0], value[1])
        ),
        "extra_poses_mm": sorted(set(groups) - required),
        "poses": pose_results,
    }


def analyze(
    fit_rows: list[dict[str, str]],
    validation_rows: list[dict[str, str]],
    allow_axis_specific_after_rig_review: bool = False,
) -> dict:
    required = required_poses()
    fit_groups = _group(fit_rows)
    fit_session_labels = {row.get("session", "") for row in fit_rows}
    validation_session_labels = {row.get("session", "") for row in validation_rows}
    session_labels_valid = fit_session_labels == {
        "fit"
    } and validation_session_labels == {"validation"}
    fit_matrix_complete = all(
        len(fit_groups.get(pose, [])) >= MIN_SAMPLES_PER_POSE for pose in required
    )
    parameters = fit_axis_depth_model(fit_rows) if fit_rows else None
    fit_results = {
        model: evaluate_model(
            fit_rows, model, parameters if model.endswith("affine") else None
        )
        for model in ("raw", "shared_depth_ratio", "axis_depth_affine")
    }
    validation_results = {
        model: evaluate_model(
            validation_rows,
            model,
            parameters if model == "axis_depth_affine" else None,
        )
        for model in ("raw", "shared_depth_ratio", "axis_depth_affine")
    }

    standard_models_pass = any(
        validation_results[name]["pass"] for name in ("raw", "shared_depth_ratio")
    )
    eligible = [
        name
        for name in ("raw", "shared_depth_ratio")
        if validation_results[name]["pass"]
    ]
    if (
        not standard_models_pass
        and allow_axis_specific_after_rig_review
        and validation_results["axis_depth_affine"]["pass"]
    ):
        eligible.append("axis_depth_affine")
    priority = {"raw": 0, "shared_depth_ratio": 1, "axis_depth_affine": 2}
    selected = min(
        eligible,
        key=lambda name: (
            validation_results[name]["worst_case_median_absolute_error_pct_of_range"],
            priority[name],
        ),
        default=None,
    )
    gate_pass = fit_matrix_complete and session_labels_valid and selected is not None
    return {
        "calibration_gate_pass": gate_pass,
        "required_pose_count": len(required),
        "samples_required_per_pose_per_session": MIN_SAMPLES_PER_POSE,
        "fit_matrix_complete": fit_matrix_complete,
        "session_labels_valid": session_labels_valid,
        "fit_session_labels": sorted(fit_session_labels),
        "validation_session_labels": sorted(validation_session_labels),
        "held_out_validation_enforced": True,
        "selected_model": selected,
        "axis_depth_model_parameters": parameters,
        "axis_specific_model_authorized_after_rig_review": (
            allow_axis_specific_after_rig_review
        ),
        "requires_rig_alignment_or_stereo_calibration_correction": (
            not standard_models_pass
        ),
        "fit_session": fit_results,
        "validation_session": validation_results,
    }


def _read(path: str) -> list[dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("validation_csv")
    parser.add_argument("--fit-csv", required=True)
    parser.add_argument("--json-out")
    parser.add_argument(
        "--allow-axis-specific-after-rig-review",
        action="store_true",
        help=(
            "Only use after checking rig alignment and rerunning OAK-D stereo "
            "calibration; the fitted model is still accepted only on held-out data."
        ),
    )
    args = parser.parse_args()
    if os.path.abspath(args.fit_csv) == os.path.abspath(args.validation_csv):
        raise SystemExit("fit and validation must be separate CSV files")
    report = analyze(
        _read(args.fit_csv),
        _read(args.validation_csv),
        allow_axis_specific_after_rig_review=(
            args.allow_axis_specific_after_rig_review
        ),
    )
    print(json.dumps(report, indent=2))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")
    sys.exit(0 if report["calibration_gate_pass"] else 1)


if __name__ == "__main__":
    main()
