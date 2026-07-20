"""Tests for the physical ArduCam smoke-test harness."""

from pathlib import Path
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

from scripts import arducam_smoke
from src.frame import CameraFrame


def _frame(width=1280, height=720):
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:, :] = (255, 0, 0)
    return CameraFrame(rgb=rgb, depth=None, rgb_down=None)


def _camera(frame=None, initialize=True):
    camera = MagicMock()
    camera.initialize_camera.return_value = initialize
    camera.capture_frame.return_value = frame or _frame()
    return camera


def test_run_smoke_test_captures_saves_and_releases(monkeypatch, tmp_path):
    camera = _camera()
    output_path = tmp_path / "results" / "frame.jpg"
    imwrite = MagicMock(return_value=True)
    monkeypatch.setattr(arducam_smoke, "ArduCam", MagicMock(return_value=camera))
    monkeypatch.setattr(arducam_smoke.cv2, "imwrite", imwrite)
    times = iter((10.0, 12.0))
    monkeypatch.setattr(arducam_smoke.time, "perf_counter", lambda: next(times))

    result = arducam_smoke.run_smoke_test(
        device_index=2,
        width=1280,
        height=720,
        frame_count=60,
        output_path=output_path,
    )

    arducam_smoke.ArduCam.assert_called_once_with(
        device_index=2, width=1280, height=720
    )
    assert camera.capture_frame.call_count == arducam_smoke.WARMUP_FRAMES + 60
    camera.stop.assert_called_once_with()
    assert result.frames_captured == 60
    assert result.width == 1280
    assert result.height == 720
    assert result.elapsed_seconds == 2.0
    assert result.frames_per_second == 30.0
    assert result.output_path == output_path.resolve()

    saved_path, saved_bgr = imwrite.call_args.args
    assert saved_path == str(output_path.resolve())
    assert np.array_equal(
        saved_bgr,
        cv2.cvtColor(camera.capture_frame.return_value.rgb, cv2.COLOR_RGB2BGR),
    )


def test_run_smoke_test_releases_camera_when_initialization_fails(monkeypatch, tmp_path):
    camera = _camera(initialize=False)
    monkeypatch.setattr(arducam_smoke, "ArduCam", MagicMock(return_value=camera))

    with pytest.raises(arducam_smoke.SmokeTestError, match="could not initialize"):
        arducam_smoke.run_smoke_test(
            device_index=99,
            width=1280,
            height=720,
            frame_count=60,
            output_path=tmp_path / "frame.jpg",
        )

    camera.capture_frame.assert_not_called()
    camera.stop.assert_called_once_with()


def test_run_smoke_test_fails_when_a_measured_capture_fails(monkeypatch, tmp_path):
    camera = _camera()
    camera.capture_frame.side_effect = [
        *[_frame() for _ in range(arducam_smoke.WARMUP_FRAMES)],
        None,
    ]
    monkeypatch.setattr(arducam_smoke, "ArduCam", MagicMock(return_value=camera))

    with pytest.raises(arducam_smoke.SmokeTestError, match="measured frame 1/60"):
        arducam_smoke.run_smoke_test(
            device_index=0,
            width=1280,
            height=720,
            frame_count=60,
            output_path=tmp_path / "frame.jpg",
        )

    camera.stop.assert_called_once_with()


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        (CameraFrame(np.empty((0, 0, 3), dtype=np.uint8), None, None), "empty"),
        (CameraFrame(np.zeros((720, 1280, 3), dtype=np.float32), None, None), "dtype"),
        (CameraFrame(np.zeros((480, 640, 3), dtype=np.uint8), None, None), "shape"),
    ],
)
def test_validate_frame_rejects_invalid_rgb_data(frame, message):
    with pytest.raises(arducam_smoke.SmokeTestError, match=message):
        arducam_smoke.validate_frame(
            frame,
            expected_width=1280,
            expected_height=720,
            label="test frame",
        )


def test_main_reports_failure(monkeypatch, capsys, tmp_path):
    def fail(**_kwargs):
        raise arducam_smoke.SmokeTestError("camera unavailable")

    monkeypatch.setattr(arducam_smoke, "run_smoke_test", fail)

    exit_code = arducam_smoke.main(["--output", str(tmp_path / "frame.jpg")])

    assert exit_code == 1
    assert "FAIL: camera unavailable" in capsys.readouterr().err


def test_positive_and_nonnegative_cli_values():
    assert arducam_smoke.positive_int("1") == 1
    assert arducam_smoke.nonnegative_int("0") == 0
    with pytest.raises(arducam_smoke.argparse.ArgumentTypeError):
        arducam_smoke.positive_int("0")
    with pytest.raises(arducam_smoke.argparse.ArgumentTypeError):
        arducam_smoke.nonnegative_int("-1")


def test_output_path_is_a_path():
    args = arducam_smoke.build_parser().parse_args([])

    assert isinstance(args.output, Path)
