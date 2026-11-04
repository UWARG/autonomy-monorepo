import json
import time

import numpy as np
import py_trees
import pytest
import rclpy
from airside_interfaces.msg import Coordinate
from camera_ros.camera_node import CameraNode
from engine import blackboard_keys
from engine.behaviors.camera.take_photo import TakePhoto
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


@pytest.mark.parametrize("sharp", [False, True])
def test_take_photo_checks_rectified_feed_and_propagates_result(tmp_path, monkeypatch, sharp):
    monkeypatch.setenv("PHOTO_DATA_DIR", str(tmp_path))
    rclpy.init(args=["--ros-args", "-p", "rectified_image_topic:=/test/image_rect"])
    camera = CameraNode()
    engine = rclpy.create_node("capture_test")
    executor = SingleThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(engine)
    behavior = TakePhoto()
    behavior.setup(node=engine)
    writer = py_trees.blackboard.Client(name="capture_test_writer")
    writer.register_key(blackboard_keys.CURRENT_WAYPOINT, py_trees.common.Access.WRITE)
    writer.set(blackboard_keys.CURRENT_WAYPOINT, Coordinate(lat=1.0, lon=2.0, alt=10.0))
    publisher = engine.create_publisher(Image, "/test/image_rect", qos_profile_sensor_data)
    imu_publisher = engine.create_publisher(Imu, "/mavros/imu/data", qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() and imu_publisher.get_subscription_count():
                break
            executor.spin_once(timeout_sec=0.02)
        rgb = np.zeros((32, 32, 3), dtype=np.uint8)
        if sharp:
            rgb[:, 16:] = 255
        stamp = engine.get_clock().now().to_msg()
        image = Image(height=32, width=32, encoding="rgb8", step=96, data=rgb.tobytes())
        image.header.stamp = stamp
        imu = Imu()
        imu.header.stamp = stamp
        imu.orientation.w = 1.0
        publisher.publish(image)
        imu_publisher.publish(imu)
        deadline = time.monotonic() + 1
        while camera._latest_rectified is None or behavior._latest_imu is None:
            assert time.monotonic() < deadline, "test messages did not reach both nodes"
            executor.spin_once(timeout_sec=0.01)
        behavior.initialise()
        status = py_trees.common.Status.RUNNING
        while status == py_trees.common.Status.RUNNING and time.monotonic() < deadline:
            status = behavior.update()
            executor.spin_once(timeout_sec=0.01)
        if sharp:
            assert status == py_trees.common.Status.SUCCESS
            record = json.loads(camera._photo_log.index_file.read_text())
            assert record["attitude"]["orientation"]["w"] == 1.0
            assert record["attitude"]["frame_offset_ns"] == 0
            assert record["sharpness"] >= 100
            assert (camera._photo_log.run_dir / record["file"]).is_file()
        else:
            assert status == py_trees.common.Status.FAILURE
            assert not list(camera._photo_log.run_dir.glob("*.png"))
            assert not camera._photo_log.index_file.exists()
        assert camera.count_publishers("/camera/image_raw") == 0
    finally:
        writer.unregister_all_keys()
        behavior.blackboard.unregister_all_keys()
        executor.shutdown()
        engine.destroy_node()
        camera.destroy_node()
        rclpy.shutdown()
