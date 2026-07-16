from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range, Imu, RCIn

from airside_interfaces.msg import FusedComms


class GroundFusionNode(Node):
    IMAGE_TOPIC = "/down/camera/image_raw"
    RANGE_TOPIC = "/mavros/rangefinder/rangefinder"
    ATTITUDE_TOPIC = "/mavros/imu/data"
    RC_TOPIC = "/mavros/rc/in"
    COMM_TOPIC = "airside_comms/info"
    PUBLISH_INTERVAL_S = 0.2

    #PARAMETER INPUTS FOR RC TRIGGERS
    _RC_TRIGGER_CHANNELS: int = 4
    _RC_TRIGGER_THRESHOLD = 1500

    def __init__(self, ) -> None:
        super().__init__("ground_fusion_node")
        self._rc_trigger_channel = self._RC_TRIGGER_CHANNELS
        self._rc_trigger_threshold = self._RC_TRIGGER_THRESHOLD
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

        self._publisher = self.create_publisher(FusedComms, self.COMM_TOPIC, 10)
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

        message = self._build_message()
        self._publisher.publish(message)
        self.get_logger().info(f"Published fused airside_comms message to '{self.COMM_TOPIC}'")

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

    def _build_message(self) -> FusedComms:
        assert self._latest_image is not None
        assert self._latest_range is not None
        assert self._latest_attitude is not None

        message = FusedComms()
        message.header.stamp = self.get_clock().now().to_msg()
        message.image = self._latest_image
        message.range = self._latest_range
        message.imu = self._latest_attitude
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GroundFusionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
