"""
Unit tests for the ArduCam USB camera.

cv2.VideoCapture is mocked out so the suite runs without real hardware.
"""

from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from src.arducam import ArduCam
from src.frame import CameraFrame


def _make_capture(is_opened=True, read_return=(True, None)):
    """Build a mock cv2.VideoCapture object."""
    capture = MagicMock()
    capture.isOpened.return_value = is_opened
    capture.read.return_value = read_return
    return capture


def _make_initialized_arducam(capture):
    """Create an ArduCam whose initialize_camera() uses the given mock capture."""
    cam = ArduCam()
    with patch("src.arducam.cv2.VideoCapture", return_value=capture):
        cam.initialize_camera()
    return cam


def test_initialize_camera_success():
    capture = _make_capture(is_opened=True)
    cam = ArduCam(device_index=2, width=640, height=480)

    with patch("src.arducam.cv2.VideoCapture", return_value=capture) as mock_video_capture:
        result = cam.initialize_camera()

    assert result is True
    assert cam._capture is capture
    mock_video_capture.assert_called_once_with(2)
    capture.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def test_initialize_camera_failure_when_device_unavailable():
    capture = _make_capture(is_opened=False)
    cam = ArduCam()

    with patch("src.arducam.cv2.VideoCapture", return_value=capture):
        result = cam.initialize_camera()

    assert result is False
    assert cam._capture is None
    capture.release.assert_called_once()


def test_capture_frame_returns_cameraframe():
    fake_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    capture = _make_capture(read_return=(True, fake_frame))
    cam = _make_initialized_arducam(capture)

    frame = cam.capture_frame()

    assert isinstance(frame, CameraFrame)
    # capture_frame() converts BGR->RGB, so rgb is a new array (not the raw frame)
    # with the same shape/dtype.
    assert frame.rgb is not fake_frame
    assert np.array_equal(frame.rgb, cv2.cvtColor(fake_frame, cv2.COLOR_BGR2RGB))
    assert frame.rgb.shape == (720, 1280, 3)
    assert frame.rgb.dtype == fake_frame.dtype
    assert frame.depth is None
    assert frame.rgb_down is None


def test_capture_frame_converts_bgr_to_rgb():
    # OpenCV delivers frames as BGR; a pure-blue pixel is [255, 0, 0] in BGR.
    bgr_frame = np.zeros((2, 2, 3), dtype=np.uint8)
    bgr_frame[:, :] = (255, 0, 0)
    capture = _make_capture(read_return=(True, bgr_frame))
    cam = _make_initialized_arducam(capture)

    frame = cam.capture_frame()

    # After BGR->RGB the same pixel must be [0, 0, 255] (blue in RGB order).
    assert np.array_equal(frame.rgb[0, 0], np.array([0, 0, 255], dtype=np.uint8))


def test_capture_frame_returns_none_on_read_failure():
    capture = _make_capture(read_return=(False, None))
    cam = _make_initialized_arducam(capture)

    assert cam.capture_frame() is None


def test_capture_frame_returns_none_when_not_initialized():
    cam = ArduCam()

    assert cam.capture_frame() is None


def test_stop_releases_capture():
    capture = _make_capture()
    cam = _make_initialized_arducam(capture)

    cam.stop()

    capture.release.assert_called_once()
    assert cam._capture is None

    # A second stop() must be safe and not raise.
    cam.stop()
