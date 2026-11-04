import pytest
import rclpy
from airside_interfaces.srv import CapturePhoto
from sensor_msgs.msg import Image

from camera_ros.camera_node import CameraNode


@pytest.fixture
def camera_node(tmp_path, monkeypatch):
    monkeypatch.setenv("PHOTO_DATA_DIR", str(tmp_path))
    rclpy.init(args=["--ros-args", "-p", "rectified_image_topic:=/test/image_rect"])
    node = CameraNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
