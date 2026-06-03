from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class AirsideCommsNode(Node):
    """Bridges the airside ROS2 system to the IMS ground station over WebSocket."""

    IMS_URL = "ws://0.0.0.0:5000/ws"

    def __init__(self) -> None:
        super().__init__("airside_comms")

    def _send(self, type: str, payload: dict) -> None:
        """Send a JSON message to the IMS ground station."""

    def _on_camera(self, msg: Image) -> None:
        """Callback for camera images, sends them to the IMS ground station."""
        
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AirsideCommsNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
