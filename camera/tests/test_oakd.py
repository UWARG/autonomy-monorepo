"""
Unit tests for OakD.
"""

import math
import sys
from unittest.mock import MagicMock

import numpy as np

from src.oakd import OakD


def test_sample_centre_depth_median_of_valid_region():
    cam = OakD()
    depth = np.zeros((10, 10), dtype=np.uint16)
    depth[3:8, 3:8] = 2000  

    assert cam.sample_centre_depth(depth) == 2.0


def test_sample_centre_depth_nan_when_all_zero():
    cam = OakD()
    depth = np.zeros((10, 10), dtype=np.uint16)

    assert math.isnan(cam.sample_centre_depth(depth))


def test_sample_centre_depth_nan_when_none():
    cam = OakD()

    assert math.isnan(cam.sample_centre_depth(None))


def test_capture_frame_returns_none_before_initialize():
    cam = OakD()

    assert cam.capture_frame() is None


def test_initialize_camera_returns_false_without_depthai(monkeypatch):
    monkeypatch.setitem(sys.modules, "depthai", None)
    cam = OakD()

    assert cam.initialize_camera() is False


def test_initialize_camera_success_with_mocked_depthai(monkeypatch):
    fake_dai = MagicMock()
    monkeypatch.setitem(sys.modules, "depthai", fake_dai)
    cam = OakD()

    assert cam.initialize_camera() is True
    fake_dai.Pipeline.return_value.start.assert_called_once()


def test_capture_frame_combines_rgb_and_depth():
    cam = OakD()
    rgb_img = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_img = np.zeros((10, 10), dtype=np.uint16)
    depth_img[3:8, 3:8] = 3000

    rgb_queue = MagicMock()
    rgb_queue.get.return_value.getCvFrame.return_value = rgb_img
    depth_queue = MagicMock()
    depth_queue.get.return_value.getCvFrame.return_value = depth_img

    cam._rgb_queue = rgb_queue
    cam._depth_queue = depth_queue

    frame = cam.capture_frame()

    assert frame is not None
    assert np.array_equal(frame.rgb, rgb_img)
    assert frame.centre_depth == 3.0


def test_close_camera_stops_and_clears_pipeline():
    cam = OakD()
    pipeline = MagicMock()
    cam._pipeline = pipeline
    cam._rgb_queue = MagicMock()
    cam._depth_queue = MagicMock()

    cam.close_camera()

    pipeline.stop.assert_called_once()
    assert cam._pipeline is None
    assert cam._rgb_queue is None
    assert cam._depth_queue is None
