"""Regression tests for camera and visualization frame pacing."""

import pytest

import frame_timer
from frame_timer import FrameTimer


@pytest.fixture
def clock(monkeypatch):
    """Provide a deterministic monotonic clock and sleeper."""
    class Clock:
        def __init__(self):
            self.now = 0.0
            self.sleeps = []

        def sleep(self, duration):
            self.sleeps.append(duration)
            self.now += duration

    result = Clock()
    monkeypatch.setattr(frame_timer.time, "monotonic", lambda: result.now)
    monkeypatch.setattr(frame_timer.time, "sleep", result.sleep)
    return result


def test_camera_processing_uses_frame_budget(clock):
    timer = FrameTimer(30)
    starts = []
    for _ in range(30):
        timer.wait()
        starts.append(clock.now)
        clock.now += 0.010
    assert starts[-1] - starts[0] == pytest.approx(29 / 30)
    assert clock.sleeps[0] == pytest.approx(1 / 30 - 0.010)


def test_slow_frames_skip_missed_slots(clock):
    timer = FrameTimer(30)
    timer.wait()
    clock.now = 0.110
    timer.wait()
    assert clock.sleeps == []
    assert not timer.ready()
    timer.wait()
    assert clock.now == pytest.approx(4 / 30)


@pytest.mark.parametrize("physics_hz", [400, 800, 1600])
def test_pose_rate_is_independent_of_physics_rate(clock, physics_hz):
    timer = FrameTimer(50)
    published = 0
    for step in range(physics_hz):
        clock.now = step / physics_hz
        published += timer.ready()
    assert published == 50


def test_pose_scheduler_does_not_sleep(clock):
    timer = FrameTimer(50)
    assert timer.ready()
    assert not timer.ready()
    assert clock.sleeps == []


@pytest.mark.parametrize("fps", [0, -1])
def test_invalid_frame_rate(fps):
    with pytest.raises(ValueError):
        FrameTimer(fps)
