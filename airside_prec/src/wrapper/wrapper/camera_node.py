from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from camera.src.abstract_camera import AbstractCamera
from camera.src.arducam import Arducam
from sensor_msgs.msg import CameraInfo
import yaml
import cv_bridge
import os
from ament_index_python.packages import get_package_share_directory


class CameraNode(Node):
    TOPIC = "camera/image"
    PUBLISH_HZ = 2.0
    WIDTH = 640
    HEIGHT = 480

    def __init__(self) -> None:
        super().__init__("camera_node")

        self._camera: AbstractCamera = Arducam()
        self._camera.initialize_camera()

        self._publisher = self.create_publisher(Image, self.TOPIC, 10)
        self._camera_info_publisher = self.create_publisher(CameraInfo, "camera/camera_info", 10)
        self._frame_id = 0
        self._bridge = cv_bridge.CvBridge()

        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)
        self.get_logger().info(
            f"Camera node ready - publishing on '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"using {type(self._camera).__name__}."
        )
        with open(os.path.join(get_package_share_directory("engine"),"camera_info.yaml"), "r") as f:
            self.camera_info = yaml.load(f, Loader=yaml.FullLoader)

    def _publish_frame(self) -> None:   
        frame = self._camera.capture_frame()
        timestamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = timestamp
        msg.header.frame_id = "camera"
        msg.encoding = "bgr8"

        if frame is None:
            msg.height = self.HEIGHT
            msg.width = self.WIDTH
            msg.step = self.WIDTH * 3
            msg.data = bytes(self.HEIGHT * self.WIDTH * 3)
        else:
            msg.height = frame.rgb.shape[0]
            msg.width = frame.rgb.shape[1]
            msg.step = frame.rgb.shape[1] * 3
            msg.data = self._bridge.cv2_to_imgmsg(frame.rgb, "bgr8").data

        cam_info = CameraInfo()
        cam_info.header.stamp = timestamp
        cam_info.header.frame_id = "camera"

        cam_info.height=self.camera_info["height"]
        cam_info.width=self.camera_info["width"]
        cam_info.distortion_model=self.camera_info["distortion_model"]
        cam_info.k=self.camera_info["camera_matrix"]["data"]
        cam_info.d=self.camera_info["distortion_coefficients"]["data"]
        cam_info.r=self.camera_info["rectification_matrix"]["data"]
        cam_info.p=self.camera_info["projection_matrix"]["data"]
        self._camera_info_publisher.publish(cam_info)
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

