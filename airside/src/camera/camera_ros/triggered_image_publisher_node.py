"""Publish the latest forward/downward images, GPS, attitude and range on a capture request."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, Range
import message_filters

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

        # Accept both best-effort MAVROS and reliable camera publishers.
        self.forward_image_sub = message_filters.Subscriber(
            self, Image, self.FORWARD_IMAGE_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.forward_image_cache = message_filters.Cache(
            self.forward_image_sub, cache_size=5,
        )

        self.downward_image_sub = message_filters.Subscriber(
            self, Image, self.DOWNWARD_IMAGE_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.downward_image_cache = message_filters.Cache(
            self.downward_image_sub, cache_size=5,
        )

        self.gps_sub = message_filters.Subscriber(
            self, NavSatFix, self.GPS_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.gps_cache = message_filters.Cache(
            self.gps_sub, cache_size=5,
        )

        self.imu_sub = message_filters.Subscriber(
            self, Imu, self.IMU_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.imu_cache = message_filters.Cache(
            self.imu_sub, cache_size=5,
        )

        self.range_sub = message_filters.Subscriber(
            self, Range, self.RANGE_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self.range_cache = message_filters.Cache(
            self.range_sub, cache_size=5,
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

    def trigger_callback(self, msg: TriggerImageCapture):
        if msg.command != 'capture':
            self.get_logger().warning(f'Unknown capture command: {msg.command!r}')
            return
        self.publish_triggered_image()

    def publish_triggered_image(self):
        """Send cached data without altering the source images or their timestamps."""
        now = self.get_clock().now()
        forward = self.forward_image_cache.getElemBeforeTime(now)
        downward = self.downward_image_cache.getElemBeforeTime(now)
        gps = self.gps_cache.getElemBeforeTime(now)
        imu = self.imu_cache.getElemBeforeTime(now)
        range_ = self.range_cache.getElemBeforeTime(now)

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
