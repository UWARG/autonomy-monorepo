"""Synthetic ROS fixtures ONLY for the isolated Docker integration test.

Publish on /dashboard_test/*, never mission/MAVROS command topics. This script
is not part of the normal dashboard or live airside launch.
"""
import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = Node("dashboard_regression_fixture")
# Keep the fixture available to late subscribers and require reliable delivery.
# This only controls the isolated regression fixture, never the real camera.
image_qos = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
image_publisher = node.create_publisher(Image, "/dashboard_test/image", image_qos)
state_publisher = node.create_publisher(State, "/dashboard_test/state", 1)


def publish():
    image = Image()
    image.header.stamp = node.get_clock().now().to_msg()
    image.header.frame_id = "synthetic-regression-fixture"
    image.width, image.height, image.step = 640, 480, 1920
    image.encoding = "rgb8"
    image.data = bytes(640 * 480 * 3)
    image_publisher.publish(image)
    state = State()
    state.header.stamp = image.header.stamp
    state.connected, state.armed, state.guided = True, False, False
    state.mode, state.system_status = "TEST_FIXTURE", 3
    state_publisher.publish(state)


node.create_timer(1.0, publish)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
