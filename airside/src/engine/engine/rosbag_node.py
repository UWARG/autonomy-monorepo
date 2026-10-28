"""ROS services and status for the IMS rosbag toggle."""

from __future__ import annotations

import json
import os

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from engine.rosbag_recorder import RosbagRecorder


class RosbagNode(Node):
    def __init__(self):
        super().__init__("ims_rosbag_controller")
        directory = self.declare_parameter(
            "output_directory", os.environ.get("ROSBAG_OUTPUT_DIR", "/ros_ws/data/rosbags")
        ).value
        # An empty array defaults to all topics; dynamic typing also permits string arrays.
        topics = self.declare_parameter(
            "topics", [], ParameterDescriptor(dynamic_typing=True)
        ).value
        if not isinstance(topics, list) or any(
            not isinstance(topic, str) or not topic.startswith("/") for topic in topics
        ):
            raise ValueError("topics must be an array of absolute ROS topic names")
        self.recorder = RosbagRecorder(directory, topics=topics)
        self.publisher = self.create_publisher(String, "/ims/rosbag/status", 10)
        self.create_service(SetBool, "/ims/rosbag/set_recording", self.set_recording)
        self.create_service(Trigger, "/ims/rosbag/get_status", self.get_status)
        self.create_timer(.5, self.publish_status)

    def set_recording(self, request, response):
        result = self.recorder.set_recording(request.data)
        response.success = result.success
        response.message = result.message
        self.publish_status()
        return response

    def get_status(self, _request, response):
        response.success = True
        response.message = json.dumps(self.recorder.status())
        return response

    def publish_status(self):
        self.publisher.publish(String(data=json.dumps(self.recorder.status())))

    def destroy_node(self):
        self.recorder.close()
        error = self.recorder.status()["error"]
        if error:
            self.get_logger().error(error)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RosbagNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
