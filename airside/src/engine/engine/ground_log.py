"""
Minimal mission-progress reporting to the GCS via MAVLink STATUSTEXT.
"""

from __future__ import annotations

import rclpy.node
from mavros_msgs.msg import StatusText
from rclpy.publisher import Publisher

_STATUSTEXT_TOPIC = "mavros/statustext/send"

# A MAVLink STATUSTEXT message carries at most 50 characters.
_MAX_TEXT_LEN = 50

_publishers: dict[rclpy.node.Node, Publisher] = {}


def send_to_ground(node: rclpy.node.Node, text: str) -> None:
    publisher = _publishers.get(node)
    if publisher is None:
        publisher = node.create_publisher(StatusText, _STATUSTEXT_TOPIC, 10)
        _publishers[node] = publisher

    message = StatusText()
    message.header.stamp = node.get_clock().now().to_msg()
    message.severity = StatusText.NOTICE
    message.text = text[:_MAX_TEXT_LEN]
    publisher.publish(message)

    node.get_logger().info(f"to ground: {text}")
