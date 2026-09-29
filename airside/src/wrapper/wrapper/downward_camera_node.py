from __future__ import annotations

import rclpy

from .camera_node import CameraNode

TOPIC = "/down/camera/image_raw"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode(topic=TOPIC, node_name="downward_camera_node")

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
