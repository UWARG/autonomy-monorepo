"""Tests for the Camera class."""

from camera import Camera


def test_camera_init(_bullet_connect, camera_obj):
    """Camera initialization stores the expected configuration."""
    camera_obj = Camera(
        attached_to_object=1, port=6000, direction=[0, 0, -1], depth_map=True
    )
    assert camera_obj.port == 6000
    assert camera_obj.direction == [0, 0, -1]
    assert camera_obj.depth_map is True
    assert camera_obj.fov == 60
    assert camera_obj.near == 1
    assert camera_obj.far == 100
    assert camera_obj.height == 224
    assert camera_obj.width == 224


def test_camera_update(_bullet_connect, camera_obj):
    """Updating the camera produces a view matrix."""
    camera_obj.update()
    assert camera_obj.view_matrix is not None


def test_camera_capture_image(_bullet_connect, camera_obj):
    """Capturing an image populates RGB and depth buffers."""
    camera_obj.capture_image()
    assert camera_obj.rgb_img is not None
    assert camera_obj.depth_img is not None
    assert camera_obj.seg_img is not None
