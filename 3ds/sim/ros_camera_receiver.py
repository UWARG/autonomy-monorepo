"""Publish only Gazebo-rendered JPEG frames received over stdin into test ROS.

Used by gazebo_camera.py through docker exec; never started by the flight stack.
"""
import base64
import json
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "73":
        raise SystemExit("This simulation adapter requires isolated ROS_DOMAIN_ID=73")
    rclpy.init()
    node = Node("gazebo_dashboard_camera")
    publisher = node.create_publisher(
        CompressedImage, "/gazebo/camera/image/compressed",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        while True:
            line = sys.stdin.buffer.readline(4_000_001)
            if not line:
                break
            if len(line) > 4_000_000 or not line.endswith(b"\n"):
                raise ValueError("Gazebo image packet exceeds limit")
            packet = json.loads(line)
            if packet.get("version") != 1:
                raise ValueError("Unsupported camera adapter schema")
            data = base64.b64decode(packet["data"], validate=True)
            if not data.startswith(b"\xff\xd8"):
                raise ValueError("Expected a JPEG rendered from Gazebo")
            message = CompressedImage()
            message.header.stamp.sec = int(packet["sec"])
            message.header.stamp.nanosec = int(packet["nanosec"])
            message.header.frame_id = "gazebo_camera"
            message.format = "jpeg"
            message.data = data
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
