from __future__ import annotations

import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from camera.src.abstract_camera import AbstractCamera
from camera.src.arducam import Arducam
from camera.src.sim import SimCamera


class CameraNode(Node):
    TOPIC = "camera/image_raw"
    PUBLISH_HZ = 50.0
    WIDTH = 1280
    HEIGHT = 720

    def __init__(self) -> None:
        super().__init__("camera_node")

        self._camera: AbstractCamera = self._create_camera()
        if not self._camera.initialize_camera():
            self.get_logger().error(
                f"Failed to initialize {type(self._camera).__name__}"
            )

        self._publisher = self.create_publisher(Image, self.TOPIC, 10)
        self._frame_id = 0
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)

        self.get_logger().info(
            f"Camera node ready - publishing on '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"using {type(self._camera).__name__}."
        )

    @staticmethod
    def _create_camera() -> AbstractCamera:
        if os.path.exists("/dev/video0"):
            return Arducam(width=CameraNode.WIDTH, height=CameraNode.HEIGHT)
        if os.path.exists("/dev/video1"):
            return Arducam(width=CameraNode.WIDTH, height=CameraNode.HEIGHT)
        return SimCamera()

    def _publish_frame(self) -> None:
        frame = self._camera.capture_frame()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.encoding = "rgb8"

        if frame is None:
            msg.height = self.HEIGHT
            msg.width = self.WIDTH
            msg.step = self.WIDTH * 3
            msg.data = bytes(self.HEIGHT * self.WIDTH * 3)
        else:
            rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2RGB)
            msg.height = rgb.shape[0]
            msg.width = rgb.shape[1]
            msg.step = rgb.shape[1] * 3
            msg.data = rgb.tobytes()

        self._publisher.publish(msg)
        self._frame_id += 1
        self.get_logger().debug(f"Published frame {self._frame_id}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
