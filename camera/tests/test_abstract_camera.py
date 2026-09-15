"""
Unit tests for AbstractCamera's threaded start/stop/run lifecycle.

Exercised through a minimal concrete FakeCamera rather than a real camera,
since the behaviour under test lives entirely in the base class.
"""

import time

import pytest

from src.abstract_camera import AbstractCamera
from src.frame import CameraFrame


def _frame() -> CameraFrame:
    return CameraFrame(rgb=None, depth=None, rgb_down=None)


class FakeCamera(AbstractCamera):
    """Concrete AbstractCamera whose behaviour is fully scripted by the test."""

    def __init__(self, *, init_results=None, capture_results=None, **kwargs):
        super().__init__(**kwargs)
        self._init_results = list(init_results) if init_results is not None else [True]
        self._capture_results = list(capture_results) if capture_results is not None else [None]
        self.init_calls = 0
        self.capture_calls = 0
        self.close_calls = 0

    def _pop(self, results: list, count_attr: str):
        idx = min(getattr(self, count_attr), len(results) - 1)
        setattr(self, count_attr, getattr(self, count_attr) + 1)
        return results[idx]

    def initialize_camera(self) -> bool:
        return self._pop(self._init_results, "init_calls")

    def capture_frame(self):
        return self._pop(self._capture_results, "capture_calls")

    def close_camera(self) -> None:
        self.close_calls += 1


def _wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def test_start_fails_when_initialize_camera_fails():
    cam = FakeCamera(init_results=[False], startup_retries=0)

    assert cam.start() is False
    assert cam.get_last_frame() is None


def test_start_succeeds_and_captures_frames():
    cam = FakeCamera(init_results=[True], capture_results=[_frame()])

    assert cam.start() is True
    assert _wait_until(lambda: cam.get_last_frame() is not None)

    cam.stop()
    assert cam.close_calls >= 1


def test_start_is_idempotent():
    cam = FakeCamera(init_results=[True], capture_results=[_frame()])

    assert cam.start() is True
    assert cam.start() is True
    assert cam.init_calls == 1

    cam.stop()


def test_stop_is_idempotent_and_safe_before_start():
    cam = FakeCamera()

    cam.stop()  # never started
    assert cam.close_calls == 0

    cam.start()
    cam.stop()
    cam.stop()
    assert cam.close_calls == 1


def test_run_raises_if_not_started():
    cam = FakeCamera()

    with pytest.raises(RuntimeError):
        cam.run()


def test_reinitializes_after_consecutive_capture_failures():
    cam = FakeCamera(
        init_results=[True, True],
        capture_results=[None, None, _frame()],
        startup_retries=1,
    )

    assert cam.start() is True
    assert _wait_until(lambda: cam.get_last_frame() is not None)
    assert cam.init_calls >= 2

    cam.stop()


def test_stops_when_reinitialize_exhausts_retries():
    cam = FakeCamera(
        init_results=[True, False, False],
        capture_results=[None],
        startup_retries=0,
    )

    assert cam.start() is True
    assert _wait_until(lambda: not cam._running)


def test_get_last_frame_reflects_most_recent_capture():
    first, second = _frame(), _frame()
    cam = FakeCamera(init_results=[True], capture_results=[first, second])

    assert cam.start() is True
    assert _wait_until(lambda: cam.get_last_frame() is second)

    cam.stop()


def test_rejects_negative_config():
    with pytest.raises(ValueError):
        FakeCamera(frame_interval_s=-1)

    with pytest.raises(ValueError):
        FakeCamera(startup_retries=-1)
