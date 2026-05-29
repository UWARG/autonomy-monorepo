from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class EngineManager(Node):
    TOPIC = "camera/image_raw"

    def __init__(self) -> None:
        super().__init__("engine_manager")

        self._subscription = self.create_subscription(
            Image,
            self.TOPIC,
            self._on_image,
            10,
        )
        self._frame_count = 0

        self.get_logger().info(
            f"Engine manager ready - listening on '{self.TOPIC}'."
        )

    def _on_image(self, msg: Image) -> None:
        self._frame_count += 1
        self.get_logger().info(
            f"Frame {self._frame_count}: {msg.width}x{msg.height} [{msg.encoding}]"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EngineManager()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
