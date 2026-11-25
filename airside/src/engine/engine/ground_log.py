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
_last_text = ""


def last_message() -> str:
    """The latest text sent to the ground, for the mission status."""
    return _last_text


def send_to_ground(node: rclpy.node.Node, text: str) -> None:
    global _last_text
    _last_text = text
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
