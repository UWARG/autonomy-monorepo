"""Publish the latest forward/downward images, GPS, attitude and range on a capture request."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, Range

from airside_interfaces.msg import TriggerImageCapture, TriggeredImageCapture


class TriggeredImagePublisherNode(Node):
    """Cache sensor messages and publish one complete bundle per capture request."""

    FORWARD_IMAGE_TOPIC = "camera/image_raw"
    DOWNWARD_IMAGE_TOPIC = "/down/camera/image_raw"
    GPS_TOPIC = "/mavros/global_position/global"
    IMU_TOPIC = "/mavros/imu/data"
    RANGE_TOPIC = "/mavros/rangefinder/rangefinder"

    def __init__(self):
        super().__init__('triggered_image_publisher')
        self.latest_forward_frame: Image | None = None
        self.latest_downward_frame: Image | None = None
        self.latest_gps: NavSatFix | None = None
        self.latest_imu: Imu | None = None
        self.latest_range: Range | None = None

        self.forward_image_subscription = self.create_subscription(
            Image, self.FORWARD_IMAGE_TOPIC, self.forward_image_callback, qos_profile_sensor_data
        )
        self.downward_image_subscription = self.create_subscription(
            Image, self.DOWNWARD_IMAGE_TOPIC, self.downward_image_callback, qos_profile_sensor_data
        )
        self.gps_subscription = self.create_subscription(
            NavSatFix, self.GPS_TOPIC, self.gps_callback, qos_profile_sensor_data
        )
        self.imu_subscription = self.create_subscription(
            Imu, self.IMU_TOPIC, self.imu_callback, qos_profile_sensor_data
        )
        self.range_subscription = self.create_subscription(
            Range, self.RANGE_TOPIC, self.range_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            TriggeredImageCapture, '/TriggeredImageCapture', 10
        )
        self.trigger_subscription = self.create_subscription(
            TriggerImageCapture, '/TriggerImageCapture', self.trigger_callback, 10
        )
        self.get_logger().info(
            'Ready for capture commands on /TriggerImageCapture; '
            'publishing bundles on /TriggeredImageCapture'
        )

    def forward_image_callback(self, msg: Image):
        self.latest_forward_frame = msg

    def downward_image_callback(self, msg: Image):
        self.latest_downward_frame = msg

    def gps_callback(self, msg: NavSatFix):
        self.latest_gps = msg

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def range_callback(self, msg: Range):
        self.latest_range = msg

    def trigger_callback(self, msg: TriggerImageCapture):
        if msg.command != 'capture':
            self.get_logger().warning(f'Unknown capture command: {msg.command!r}')
            return
        self.publish_triggered_image()

    def publish_triggered_image(self):
        """Send cached data without altering the source images or their timestamps."""
        forward, downward, gps, imu, range_ = (
            self.latest_forward_frame,
            self.latest_downward_frame,
            self.latest_gps,
            self.latest_imu,
            self.latest_range,
        )
        if forward is None or downward is None or gps is None or imu is None or range_ is None:
            self.get_logger().warning(
                'Capture skipped: waiting for forward camera, downward camera, GPS, IMU '
                'and rangefinder data. Send another trigger when all inputs are available.'
            )
            return

        message = TriggeredImageCapture()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = forward.header.frame_id
        message.forward_image = forward
        message.downward_image = downward
        message.location.lat = gps.latitude
        message.location.lon = gps.longitude
        message.location.alt = gps.altitude
        message.imu = imu
        message.range = range_
        self.publisher.publish(message)
        self.get_logger().info('Published triggered image with GPS, orientation and range')


def main(args=None):
    rclpy.init(args=args)
    node = TriggeredImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
