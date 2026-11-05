"""Numerical ground-truth and trial-file failure tests; no camera required."""

import csv
import json
import math
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from src.vio import VioPose
from tools import analyze_vio, check_vio, record_vio


def samples(end_position=(0, 0, 0), end_q=(0, 0, 0, 1)):
    return [
        (0, (0, 0, 0), (0, 0, 0, 1)),
        (1, (0, 0, 0), (0, 0, 0, 1)),
        (3, end_position, end_q),
        (4, end_position, end_q),
    ]


def test_translation_error_uses_measured_ground_truth():
    result = analyze_vio.analyze(
        samples((3, 4, 0)),
        trial="translation",
        expected_distance_m=4,
        position_tolerance_m=0.5,
    )
    assert result["metrics"]["endpoint_displacement_m"] == 5
    assert result["metrics"]["translation_distance_error_m"] == 1
    assert result["assessment"] == "fail"


def test_stationary_and_loop_metrics():
    data = samples()
    data.insert(2, (2, (3, 4, 0), (0, 0, 0, 1)))
    stationary = analyze_vio.analyze(data, trial="stationary", position_tolerance_m=0.1)
    loop = analyze_vio.analyze(
        data, trial="closed-loop", path_length_m=10, position_tolerance_m=0.1
    )
    assert stationary["assessment"] == "fail"
    assert stationary["metrics"]["maximum_distance_from_start_m"] == 5
    assert loop["assessment"] == "pass"
    assert loop["metrics"]["endpoint_drift_percent_of_measured_path"] == 0


def test_rotation_angle_and_quaternion_sign():
    q = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
    result = analyze_vio.analyze(
        samples(end_q=q),
        trial="rotation",
        expected_angle_deg=90,
        position_tolerance_m=0.1,
        angle_tolerance_deg=1,
    )
    assert result["metrics"]["endpoint_rotation_deg"] == pytest.approx(90)
    assert result["assessment"] == "pass"
    assert analyze_vio.quaternion_angle(q, tuple(-v for v in q)) == pytest.approx(0)


def test_no_tolerances_does_not_claim_pass():
    assert (
        analyze_vio.analyze(samples(), trial="stationary")["assessment"]
        == "not_assessed"
    )
    result = analyze_vio.analyze(
        samples(), trial="rotation", expected_angle_deg=0, position_tolerance_m=1
    )
    assert result["assessment"] == "not_assessed"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"trial": "translation"},
        {"trial": "rotation"},
        {"trial": "stationary", "endpoint_window_s": 2},
        {"trial": "closed-loop", "path_length_m": 0},
        {"trial": "rotation", "expected_angle_deg": 360},
        {"trial": "stationary", "position_tolerance_m": float("nan")},
    ],
)
def test_invalid_analysis_arguments(kwargs):
    with pytest.raises(ValueError):
        analyze_vio.analyze(samples(), **kwargs)


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(record_vio.CSV_FIELDS)
        for t, p, q in rows:
            writer.writerow([t, t, "", *p, *q])


def test_csv_and_analysis_cli(tmp_path, capsys):
    path = tmp_path / "trial.csv"
    report = tmp_path / "report.json"
    write_csv(path, samples())
    assert (
        analyze_vio.main([str(path), "--trial", "stationary", "--output", str(report)])
        == 0
    )
    assert json.loads(report.read_text())["assessment"] == "not_assessed"
    capsys.readouterr()
    with pytest.raises(SystemExit):
        analyze_vio.main([str(path), "--trial", "stationary", "--output", str(report)])


@pytest.mark.parametrize(
    "rows",
    [
        [],
        samples()[:1],
        [(0, (0, 0, 0), (0, 0, 0, 1)), (0, (0, 0, 0), (0, 0, 0, 1))],
        [(0, (0, 0, 0), (0, 0, 0, 1)), (1, (float("nan"), 0, 0), (0, 0, 0, 1))],
        [(0, (0, 0, 0), (0, 0, 0, 1)), (1, (0, 0, 0), (0, 0, 0, 0))],
    ],
)
def test_invalid_csv_is_rejected(tmp_path, rows):
    path = tmp_path / "bad.csv"
    write_csv(path, rows)
    with pytest.raises(ValueError):
        analyze_vio.load_samples(path)


class Clock:
    def __init__(self):
        self.t = 10.0

    def monotonic(self):
        return self.t

    def sleep(self, duration):
        self.t += duration


@pytest.fixture
def recorder(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(record_vio.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(record_vio.time, "sleep", clock.sleep)

    class Camera:
        closed = False

        def __init__(self, **kwargs):
            pass

        def initialize_camera(self):
            return True

        def capture_pose(self):
            return VioPose((1, 2, 3), (0, 0, 0, 1), None, clock.monotonic())

        def close_camera(self):
            Camera.closed = True

    monkeypatch.setattr(record_vio, "OakD", Camera)
    return Camera


def test_recording_and_metadata_round_trip(tmp_path, recorder):
    path = tmp_path / "trial.csv"
    assert (
        record_vio.main(["--output", str(path), "--duration", "3", "--warmup", "0"])
        == 0
    )
    metadata = json.loads(path.with_suffix(".metadata.json").read_text())
    assert metadata["status"] == "complete"
    loaded = analyze_vio.load_samples(path)
    assert len(loaded) == metadata["samples"]
    assert (
        analyze_vio.analyze(loaded, trial="stationary")["metrics"][
            "endpoint_displacement_m"
        ]
        == 0
    )
    assert recorder.closed


def test_recorder_rejects_overwriting_trial(tmp_path, recorder):
    path = tmp_path / "existing.csv"
    path.write_text("preserve me")
    with pytest.raises(SystemExit):
        record_vio.main(["--output", str(path)])
    assert path.read_text() == "preserve me"


def test_initialization_failure_is_recorded(tmp_path, recorder, monkeypatch):
    monkeypatch.setattr(recorder, "initialize_camera", lambda self: False)
    path = tmp_path / "failed.csv"
    assert record_vio.main(["--output", str(path)]) == 1
    assert (
        json.loads(path.with_suffix(".metadata.json").read_text())["status"] == "failed"
    )
    assert recorder.closed


def test_silence_timeout_fails_trial(tmp_path, recorder, monkeypatch):
    monkeypatch.setattr(recorder, "capture_pose", lambda self: None)
    path = tmp_path / "silent.csv"
    assert (
        record_vio.main(
            ["--output", str(path), "--warmup", "0", "--max-silence", "0.02"]
        )
        == 1
    )
    assert (
        "silence timeout"
        in json.loads(path.with_suffix(".metadata.json").read_text())["error"]
    )


def test_interrupt_during_warmup_preserves_manifest(tmp_path, recorder, monkeypatch):
    def interrupt(self):
        raise KeyboardInterrupt

    monkeypatch.setattr(recorder, "capture_pose", interrupt)
    path = tmp_path / "interrupted.csv"
    assert record_vio.main(["--output", str(path)]) == 130
    assert (
        json.loads(path.with_suffix(".metadata.json").read_text())["status"]
        == "interrupted"
    )
    assert recorder.closed


def test_sdk_preflight_does_not_open_device(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "depthai",
        SimpleNamespace(__version__="test", node=SimpleNamespace(BasaltVIO=object())),
    )
    camera = Mock()
    monkeypatch.setattr(check_vio, "OakD", camera)
    assert check_vio.main([]) == 0
    camera.assert_not_called()


def test_sdk_preflight_rejects_missing_basalt(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "depthai",
        SimpleNamespace(__version__="test", node=SimpleNamespace()),
    )
    assert check_vio.main([]) == 1


def test_hardware_preflight_requires_frames_and_poses(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "depthai",
        SimpleNamespace(__version__="test", node=SimpleNamespace(BasaltVIO=object())),
    )
    camera = Mock()
    camera.initialize_camera.return_value = True
    monkeypatch.setattr(check_vio, "OakD", Mock(return_value=camera))
    assert check_vio.main(["--hardware"]) == 0
    assert camera.capture_pose.call_count >= 2
    assert camera.capture_frame.call_count >= 2
    camera.close_camera.assert_called_once()


def test_hardware_preflight_closes_on_start_failure(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "depthai",
        SimpleNamespace(__version__="test", node=SimpleNamespace(BasaltVIO=object())),
    )
    camera = Mock()
    camera.initialize_camera.return_value = False
    monkeypatch.setattr(check_vio, "OakD", Mock(return_value=camera))
    assert check_vio.main(["--hardware"]) == 1
    camera.close_camera.assert_called_once()
