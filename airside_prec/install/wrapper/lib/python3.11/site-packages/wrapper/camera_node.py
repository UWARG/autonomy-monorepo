from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from camera.src.abstract_camera import AbstractCamera
from camera.src.sim import SimCamera
from sensor_msgs.msg import CameraInfo


class CameraNode(Node):
    TOPIC = "camera/image"
    PUBLISH_HZ = 2.0
    WIDTH = 640
    HEIGHT = 480

    def __init__(self) -> None:
        super().__init__("camera_node")

        self._camera: AbstractCamera = SimCamera()
        self._camera.initialize_camera()

        self._publisher = self.create_publisher(Image, self.TOPIC, 10)
        self._camera_info_publisher = self.create_publisher(CameraInfo, "camera/camera_info", 10)
        self._frame_id = 0

        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)
        self.get_logger().info(
            f"Camera node ready - publishing on '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"using {type(self._camera).__name__}."
        )

    def _publish_frame(self) -> None:   
        frame = self._camera.capture_frame()
        timestamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = timestamp
        msg.header.frame_id = "camera"
        msg.encoding = "rgb8"

        if frame is None:
            msg.height = self.HEIGHT
            msg.width = self.WIDTH
            msg.step = self.WIDTH * 3
            msg.data = bytes(self.HEIGHT * self.WIDTH * 3)
        else:
            msg.height = frame.rgb.shape[0]
            msg.width = frame.rgb.shape[1]
            msg.step = frame.rgb.shape[1] * 3
            msg.data = frame.rgb.tobytes()

        camera_info = CameraInfo()
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = "camera"
        """
        camera_info.width = self.WIDTH
        camera_info.height = self.HEIGHT
        camera_info.distortion_model = "plumb_bob"
        camera_info.K = [self.WIDTH/2,0,self.WIDTH/2,0,self.HEIGHT/2,self.HEIGHT/2,0,0,1]
        camera_info.D = [0,0,0,0]
        self._camera_info_publisher.publish(camera_info)
        """
        self._camera_info_publisher.publish(camera_info)
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

