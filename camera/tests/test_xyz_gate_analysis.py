import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_xyz_gate.py"
SPEC = importlib.util.spec_from_file_location("analyze_xyz_gate", SCRIPT)
ANALYZER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ANALYZER)


def rows(
    raw_offset=0.0,
    calibrated_offset=0.0,
    omit=None,
    session="fit",
):
    result = []
    for x, y, z in ANALYZER.required_poses():
        if (x, y, z) == omit:
            continue
        for index in range(ANALYZER.MIN_SAMPLES_PER_POSE):
            result.append(
                {
                    "session": session,
                    "gt_x_mm": str(x),
                    "gt_y_mm": str(y),
                    "gt_z_mm": str(z),
                    "raw_x_mm": str(x + raw_offset),
                    "raw_y_mm": str(y + raw_offset),
                    "raw_z_mm": str(z + raw_offset),
                    "cal_x_mm": str(x + calibrated_offset),
                    "cal_y_mm": str(y + calibrated_offset),
                    "cal_z_mm": str(z + calibrated_offset),
                    "sample": str(index),
                }
            )
    return result


def test_true_per_frame_median_absolute_error_not_error_of_median():
    samples = []
    for value in (-10.0, 10.0):
        for _ in range(50):
            samples.append(
                {
                    "raw_x_mm": str(value),
                    "cal_x_mm": str(value),
                }
            )
    stats = ANALYZER._axis_stats(
        samples,
        "x",
        truth=0.0,
        range_mm=1000.0,
        predict=lambda row, axis: float(row[f"cal_{axis}_mm"]),
    )
    assert stats["median_absolute_error_mm"] == 10.0


def test_requires_all_25_poses_in_both_sessions():
    fit = rows()
    validation = rows(
        omit=(0.0, 0.0, 3000.0),
        session="validation",
    )
    report = ANALYZER.analyze(fit, validation)
    assert report["required_pose_count"] == 25
    assert not report["calibration_gate_pass"]
    assert report["validation_session"]["raw"]["missing_poses_mm"]


def test_model_selection_favors_lower_held_out_error_and_raw_ties():
    report = ANALYZER.analyze(
        rows(raw_offset=0.0, calibrated_offset=20.0),
        rows(
            raw_offset=0.0,
            calibrated_offset=20.0,
            session="validation",
        ),
    )
    assert report["calibration_gate_pass"]
    assert report["selected_model"] == "raw"

    shared_better = ANALYZER.analyze(
        rows(raw_offset=40.0, calibrated_offset=0.0),
        rows(
            raw_offset=40.0,
            calibrated_offset=0.0,
            session="validation",
        ),
    )
    assert shared_better["selected_model"] == "shared_depth_ratio"


def test_fit_session_is_required_even_when_validation_passes():
    incomplete_fit = rows(omit=(0.0, 0.0, 1000.0))
    report = ANALYZER.analyze(
        incomplete_fit,
        rows(session="validation"),
    )
    assert not report["fit_matrix_complete"]
    assert not report["calibration_gate_pass"]


def test_axis_specific_fallback_requires_explicit_rig_review_authorization():
    fit = rows(raw_offset=400.0, calibrated_offset=400.0)
    validation = rows(
        raw_offset=400.0,
        calibrated_offset=400.0,
        session="validation",
    )
    blocked = ANALYZER.analyze(fit, validation)
    assert blocked["selected_model"] is None
    assert blocked["requires_rig_alignment_or_stereo_calibration_correction"]

    allowed = ANALYZER.analyze(
        fit,
        validation,
        allow_axis_specific_after_rig_review=True,
    )
    assert allowed["calibration_gate_pass"]
    assert allowed["selected_model"] == "axis_depth_affine"
