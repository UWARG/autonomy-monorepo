"""
Publishes a consistent heartbeat to the ground station over rosbridge.
""" 
from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class HeartbeatNode(Node):
    TOPIC = "heartbeat"
    PUBLISH_HZ = 1.0

    def __init__(self) -> None:
        super().__init__("heartbeat_node")

        self._publisher = self.create_publisher(Empty, self.TOPIC, 10)
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_heartbeat)

        self.get_logger().info(
            f"Heartbeat node ready - publishing on '{self.TOPIC}' at {self.PUBLISH_HZ} Hz."
        )
    def _publish_heartbeat(self) -> None:
        self._publisher.publish(Empty())

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HeartbeatNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
