"""Publish temperatures measured on the machine running this ROS node."""
from __future__ import annotations

import json
import socket
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dashboard_monitor.sensors import read_temperatures


class TemperatureNode(Node):
    def __init__(self):
        super().__init__("dashboard_temperatures")
        self.declare_parameter("sys_root", "/sys")
        self._publisher = self.create_publisher(String, "/dashboard/temperatures", 1)
        self.create_timer(2.0, self._publish)

    def _publish(self):
        sys_root = Path(self.get_parameter("sys_root").value).resolve()
        readings = read_temperatures(sys_root)
        payload = {
            "version": 1, "stamp": self.get_clock().now().nanoseconds / 1e9,
            "host": socket.gethostname(),
            "source": f"Linux sensor files at {sys_root} on monitoring-node host",
            **readings,
        }
        self._publisher.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
