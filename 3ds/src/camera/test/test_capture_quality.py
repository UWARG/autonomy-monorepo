import json

import numpy as np
import pytest
import rclpy
from airside_interfaces.srv import CapturePhoto
from sensor_msgs.msg import Image, Imu

from camera_ros.camera_node import CameraNode


def make_camera(args, tmp_path, monkeypatch):
    monkeypatch.setenv("PHOTO_DATA_DIR", str(tmp_path))
    overrides = ["-p", "rectified_image_topic:=/test/image_rect"]
    for param in args:
        overrides += ["-p", param]
    rclpy.init(args=["--ros-args"] + overrides)
    return CameraNode()


@pytest.fixture
def camera_node(tmp_path, monkeypatch):
    node = make_camera([], tmp_path, monkeypatch)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def sharp_frame(node):
    """Build a fresh-stamped 16x16 checkerboard with high Laplacian variance."""
    rgb = np.indices((16, 16, 3)).sum(axis=0) % 2 * 255
    image = Image(
        height=16, width=16, encoding="rgb8", step=48,
        data=rgb.astype(np.uint8).tobytes(),
    )
    image.header.stamp = node.get_clock().now().to_msg()
    return image


def frame_stamp_ns(node):
    return node._latest_rectified[1]


def feed_frame(node, image):
    node._on_rectified_image(image)


def imu_at(stamp_ns):
    """Return an Imu stamped at exactly ``stamp_ns`` nanoseconds."""
    imu = Imu()
    imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    imu.orientation.w = 1.0
    return imu


def test_missing_rectified_feed_does_not_fall_back(camera_node):
    response = camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())
    assert not response.success
    assert not response.path


def test_old_frame_is_rejected(camera_node):
    image = Image()
    image.header.stamp = camera_node.get_clock().now().to_msg()
    image.header.stamp.sec -= 2
    camera_node._on_rectified_image(image)
    response = camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())
    assert not response.success
    assert "stale" in response.message


def test_valid_quality_parameters_pass_through(tmp_path, monkeypatch):
    node = make_camera(["min_sharpness:=250.0"], tmp_path, monkeypatch)
    try:
        assert node._min_sharpness == 250.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_quality_parameters_are_clamped_not_raised(tmp_path, monkeypatch):
    # A negative sharpness floor, a frame age past the ceiling, and a
    # negative IMU window must warn and clamp, not crash the node.
    node = make_camera(
        [
            "min_sharpness:=-5.0",
            "max_frame_age_s:=100.0",
            "max_imu_offset_s:=-0.1",
        ],
        tmp_path,
        monkeypatch,
    )
    try:
        assert node._min_sharpness == 100.0  # below range -> default
        assert node._max_frame_age_s == 5.0  # above ceiling -> clamped
        assert node._max_imu_offset_s == 0.25  # below range -> default
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "value", [float("nan"), float("inf"), float("-inf"), -1, 0, "100", None]
)
def test_bounded_parameter_accepts_only_finite_positive(camera_node, value):
    # The helper itself never raises, whatever the parameter server hands it.
    assert camera_node._bounded_parameter("test_param", value, 5.0, 0.5) in (0.5, 5.0)


def test_malformed_frame_propagates_instead_of_counting_as_rejection(camera_node):
    # A wrong encoding is malformed input, not an ordinary low-quality
    # rejection: it stays visible as an exception instead of being swallowed
    # into a "rejected" response.
    image = Image(encoding="bgra8", height=4, width=4, step=16, data=bytes(4 * 16))
    image.header.stamp = camera_node.get_clock().now().to_msg()
    camera_node._on_rectified_image(image)
    with pytest.raises(ValueError):
        camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())


def test_capture_without_imu_samples_records_no_attitude(camera_node):
    feed_frame(camera_node, sharp_frame(camera_node))
    response = camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())
    assert response.success
    record = json.loads(camera_node._photo_log.index_file.read_text())
    assert record["attitude"] is None


def test_stale_imu_stream_is_not_attached(camera_node):
    # MAVROS stopped publishing two seconds before the frame: the newest
    # sample is still too old to describe it, so no attitude is recorded.
    feed_frame(camera_node, sharp_frame(camera_node))
    camera_node._on_imu(imu_at(frame_stamp_ns(camera_node) - 2_000_000_000))
    response = camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())
    assert response.success
    record = json.loads(camera_node._photo_log.index_file.read_text())
    assert record["attitude"] is None


def test_unstamped_imu_sample_is_ignored(camera_node):
    camera_node._on_imu(Imu())
    feed_frame(camera_node, sharp_frame(camera_node))
    response = camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())
    assert response.success
    record = json.loads(camera_node._photo_log.index_file.read_text())
    assert record["attitude"] is None


def test_attitude_sample_closest_to_the_frame_wins(camera_node):
    feed_frame(camera_node, sharp_frame(camera_node))
    frame_ns = frame_stamp_ns(camera_node)
    camera_node._on_imu(imu_at(frame_ns - 150_000_000))
    camera_node._on_imu(imu_at(frame_ns - 5_000_000))
    response = camera_node._on_capture(CapturePhoto.Request(), CapturePhoto.Response())
    assert response.success
    record = json.loads(camera_node._photo_log.index_file.read_text())
    # The 5 ms-old sample describes the frame better than the 150 ms one.
    assert record["attitude"]["frame_offset_ns"] == -5_000_000
