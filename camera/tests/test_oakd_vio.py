"""Hardware-free tests for the OAK-D VIO interface."""

from collections import deque
from datetime import timedelta
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from src.oakd import OakD, VIO_QUEUE_SIZE


def make_message(
    position=(1.0, 2.0, 3.0),
    rotation=(0.0, 0.0, 0.0, 1.0),
    timestamp_s=1.5,
):
    """Create a message with the same pose accessors as the SDK."""
    return SimpleNamespace(
        getTranslation=lambda: SimpleNamespace(
            x=position[0],
            y=position[1],
            z=position[2],
        ),
        getQuaternion=lambda: SimpleNamespace(
            qx=rotation[0],
            qy=rotation[1],
            qz=rotation[2],
            qw=rotation[3],
        ),
        getTimestamp=lambda: timedelta(seconds=timestamp_s),
    )


class FakeQueue:
    def __init__(self, messages):
        self.messages = deque(messages)
        self.calls = 0

    def tryGet(self):
        self.calls += 1
        if not self.messages:
            return None
        return self.messages.popleft()


def camera_with_messages(messages):
    camera = OakD(enable_vio=True)
    camera._vio_queue = FakeQueue(messages)
    return camera


def test_vio_is_disabled_by_default():
    camera = OakD()

    assert camera._enable_vio is False
    assert camera.capture_pose() is None


def test_empty_queue_returns_none():
    camera = camera_with_messages([])

    assert camera.capture_pose() is None


def test_pose_contains_position_orientation_and_timestamp():
    camera = camera_with_messages([make_message()])

    pose = camera.capture_pose()

    assert pose is not None
    assert pose.position_m == (1.0, 2.0, 3.0)
    assert pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert pose.timestamp_s == 1.5
    assert pose.received_monotonic_s > 0


def test_quaternion_is_normalized():
    camera = camera_with_messages(
        [make_message(rotation=(0.0, 0.0, 0.0, 2.0))]
    )

    pose = camera.capture_pose()

    assert pose is not None
    assert pose.orientation_xyzw == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )


@pytest.mark.parametrize("timestamp_s", [0.0, -1.0])
def test_unset_or_negative_timestamp_becomes_none(timestamp_s):
    camera = camera_with_messages(
        [make_message(timestamp_s=timestamp_s)]
    )

    pose = camera.capture_pose()

    assert pose is not None
    assert pose.timestamp_s is None


def test_newest_queued_pose_is_returned():
    camera = camera_with_messages(
        [
            make_message(position=(1.0, 0.0, 0.0)),
            make_message(position=(2.0, 0.0, 0.0)),
            make_message(position=(3.0, 0.0, 0.0)),
        ]
    )

    pose = camera.capture_pose()

    assert pose is not None
    assert pose.position_m == (3.0, 0.0, 0.0)
    assert camera.capture_pose() is None


@pytest.mark.parametrize(
    "message",
    [
        make_message(position=(float("nan"), 0.0, 0.0)),
        make_message(position=(float("inf"), 0.0, 0.0)),
        make_message(rotation=(0.0, 0.0, 0.0, 0.0)),
        make_message(rotation=(float("nan"), 0.0, 0.0, 1.0)),
        object(),
    ],
)
def test_invalid_message_is_skipped(message):
    camera = camera_with_messages(
        [
            message,
            make_message(position=(4.0, 5.0, 6.0)),
        ]
    )

    pose = camera.capture_pose()

    assert pose is not None
    assert pose.position_m == (4.0, 5.0, 6.0)


def test_polling_work_is_bounded():
    camera = camera_with_messages(
        [make_message()] * (VIO_QUEUE_SIZE + 5)
    )

    camera.capture_pose()

    assert camera._vio_queue.calls == VIO_QUEUE_SIZE
    assert len(camera._vio_queue.messages) == 5


def test_queue_failure_returns_none(caplog):
    camera = OakD(enable_vio=True)
    camera._vio_queue = Mock()
    camera._vio_queue.tryGet.side_effect = RuntimeError(
        "Device disconnected"
    )

    assert camera.capture_pose() is None
    assert "Failed to read VIO queue" in caplog.text


def test_close_stops_pipeline_and_clears_queues():
    camera = camera_with_messages([make_message()])
    pipeline = Mock()
    camera._pipeline = pipeline
    camera._rgb_queue = Mock()
    camera._depth_queue = Mock()

    camera.close_camera()

    pipeline.stop.assert_called_once()
    assert camera._pipeline is None
    assert camera._rgb_queue is None
    assert camera._depth_queue is None
    assert camera._vio_queue is None
    assert camera.capture_pose() is None


def test_close_clears_queues_even_if_stop_fails(caplog):
    camera = camera_with_messages([make_message()])
    camera._pipeline = Mock()
    camera._pipeline.stop.side_effect = RuntimeError(
        "Device disconnected"
    )

    camera.close_camera()

    assert camera._pipeline is None
    assert camera._vio_queue is None
    assert "Failed to stop OAK-D pipeline" in caplog.text