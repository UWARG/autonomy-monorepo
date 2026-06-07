from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray



class EngineManager(Node):
    TOPIC = "camera/image_raw"

    def __init__(self) -> None:
        super().__init__("engine_manager")
        self.apriltag_subscriber = self.create_subscription(AprilTagDetectionArray, "apriltags/detections", self.apriltag_callback, 10)
        self._frame_count = 0

        self.get_logger().info(
            f"Engine manager ready - listening on '{self.TOPIC}'."
        )

    def apriltag_callback(self, msg: AprilTagDetectionArray) -> None:
        self.get_logger().info(f"Apriltag detected: {msg.detections}")



def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EngineManager()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
