from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range, Imu, RCIn
from std_msgs.msg import String


class GroundFusionNode(Node):
    IMAGE_TOPIC = "/down/camera/image_raw"
    RANGE_TOPIC = "/mavros/rangefinder/rangefinder"
    ATTITUDE_TOPIC = "/mavros/imu/data"
    RC_TOPIC = "/mavros/rc/in"
    COMM_TOPIC = "airside_comms/info"
    FUSED_IMAGE_TOPIC = "airside_comms/image"
    PUBLISH_INTERVAL_S = 0.2

    #PARAMETER INPUTS FOR RC TRIGGERS 
    RC_TRIGGER_CHANNELS: int = 4
    RC_TRIGGER_THRESHOLD = 1500

    def __init__(self, ) -> None:
        super().__init__("ground_fusion_node")
        self.get_logger().info(
            f"RC trigger configured channel={self._rc_trigger_channel} threshold={self._rc_trigger_threshold}"
        )
        self._last_triggered_channels: list[int] = []
        self._latest_image: Image | None = None
        self._latest_range: Range | None = None
        self._latest_attitude: Imu | None = None
        self._latest_rc: RCIn | None = None

        self._image_sub = self.create_subscription(
            Image,
            self.IMAGE_TOPIC,
            self._image_callback,
            10,
        )
        self._range_sub = self.create_subscription(
            Range,
            self.RANGE_TOPIC,
            self._range_callback,
            10,
        )
        self._attitude_sub = self.create_subscription(
            Imu,
            self.ATTITUDE_TOPIC,
            self._attitude_callback,
            10,
        )
        self._rc_sub = self.create_subscription(
            RCIn,
            self.RC_TOPIC,
            self._rc_callback,
            10,
        )

        self._publisher = self.create_publisher(String, self.COMM_TOPIC, 10)
        self._image_publisher = self.create_publisher(Image, self.FUSED_IMAGE_TOPIC, 10)
        self.create_timer(self.PUBLISH_INTERVAL_S, self._publish_if_triggered)

        self.get_logger().info(
            "Ground fusion node ready - subscribed to image, range, attitude, rc and publishing airside_comms."
        )

    def _image_callback(self, msg: Image) -> None:
        self._latest_image = msg
        self.get_logger().debug("Received image message")

    def _range_callback(self, msg: Range) -> None:
        self._latest_range = msg
        self.get_logger().debug("Received rangefinder message")

    def _attitude_callback(self, msg: Imu) -> None:
        self._latest_attitude = msg
        self.get_logger().debug("Received attitude message")

    def _rc_callback(self, msg: RCIn) -> None:
        self._latest_rc = msg
        self.get_logger().debug("Received RC input message")

    def _publish_if_triggered(self) -> None:
        if not self._rc_triggered():
            self.get_logger().debug("RC trigger is not engaged; skipping publish.")
            return

        if not self._all_data_ready():
            self.get_logger().debug("Waiting for all sensor data before publishing.")
            return

        payload = self._build_payload()
        message = String(data=json.dumps(payload))
        self._publisher.publish(message)
        self.get_logger().info("Published fused airside_comms payload")

        if self._latest_image is not None:
            self._image_publisher.publish(self._latest_image)
            self.get_logger().info(
                f"Published triggered image frame to '{self.FUSED_IMAGE_TOPIC}'"
            )

    def _rc_triggered(self) -> bool:
        if self._latest_rc is None:
            return False

        channels = self._latest_rc.channels

        triggered_idxs: list[int] = []
        # check the configured channel
        if 0 <= self._rc_trigger_channel < len(channels):
            val = channels[self._rc_trigger_channel]
            if val >= self._rc_trigger_threshold:
                triggered_idxs.append(self._rc_trigger_channel)

        # store for payload
        self._last_triggered_channels = triggered_idxs
        return len(triggered_idxs) > 0

    def _all_data_ready(self) -> bool:
        return (
            self._latest_image is not None
            and self._latest_range is not None
            and self._latest_attitude is not None
            and self._latest_rc is not None
        )

    def _build_payload(self) -> dict[str, object]:
        assert self._latest_image is not None
        assert self._latest_range is not None
        assert self._latest_attitude is not None
        assert self._latest_rc is not None

        return {
            # sensor_msgs/Range message fields
            # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html
            "range": {
                "header": {
                    "frame_id": self._latest_range.header.frame_id,
                    "stamp": self._latest_range.header.stamp.sec,
                },
                "radiation_type": int(self._latest_range.radiation_type),
                "field_of_view": self._latest_range.field_of_view,
                "min_range": self._latest_range.min_range,
                "max_range": self._latest_range.max_range,
                "range": self._latest_range.range,
            },
            # sensor_msgs/Imu message fields
            # https://docs.ros.org/en/humble/api/sensor_msgs/html/msg/Imu.html
            "imu": {
                "header": {
                    "stamp": self._latest_attitude.header.stamp.sec,
                    "frame_id": self._latest_attitude.header.frame_id,
                },
                "orientation": {
                    "x": self._latest_attitude.orientation.x,
                    "y": self._latest_attitude.orientation.y,
                    "z": self._latest_attitude.orientation.z,
                    "w": self._latest_attitude.orientation.w,
                },
                "orientation_covariance": self._latest_attitude.orientation_covariance,
                "angular_velocity": {
                    "x": self._latest_attitude.angular_velocity.x,
                    "y": self._latest_attitude.angular_velocity.y,
                    "z": self._latest_attitude.angular_velocity.z,
                },
                "angular_velocity_covariance": self._latest_attitude.angular_velocity_covariance,
                "linear_acceleration": {
                    "x": self._latest_attitude.linear_acceleration.x,
                    "y": self._latest_attitude.linear_acceleration.y,
                    "z": self._latest_attitude.linear_acceleration.z,
                },
                "linear_acceleration_covariance": self._latest_attitude.linear_acceleration_covariance,
            },
            "rc": {
                "triggered": len(self._last_triggered_channels) > 0,
                "triggered_channels": self._last_triggered_channels,
                "triggered_values": [
                    int(self._latest_rc.channels[i])
                    for i in self._last_triggered_channels
                    if i < len(self._latest_rc.channels)
                ],
            },
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GroundFusionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
