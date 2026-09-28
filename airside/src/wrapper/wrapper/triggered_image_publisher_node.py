"""Publish the latest image, GPS location and IMU on a capture request."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
import message_filters

from airside_interfaces.msg import TriggerImageCapture, TriggeredImageCapture


class TriggeredImagePublisherNode(Node):
    """Cache sensor messages and publish one complete bundle per capture request."""

    def __init__(self):
        super().__init__('triggered_image_publisher')
        self.latest_frame: Image | None = None
        self.latest_gps: NavSatFix | None = None
        self.latest_imu: Imu | None = None

        # Accept both best-effort MAVROS and reliable camera publishers.
        self.image_sub = message_filters.Subscriber(
            self, Image, '/camera/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.image_cache = message_filters.Cache(
            self.image_sub, cache_size=5,
        )

        self.gps_sub = message_filters.Subscriber(
            self, NavSatFix, '/mavros/global_position/global',
            qos_profile=qos_profile_sensor_data,
        )
        self.gps_cache = message_filters.Cache(
            self.gps_sub, cache_size=5,
        )

        self.imu_sub = message_filters.Subscriber(
            self, Imu, '/mavros/imu/data',
            qos_profile=qos_profile_sensor_data,
        )
        self.imu_cache = message_filters.Cache(
            self.imu_sub, cache_size=5,
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
        """Send cached data without altering the source image or its timestamp."""

        frame = self.image_cache.getElemBeforeTime(self.get_clock().now())
        gps = self.gps_cache.getElemBeforeTime(self.get_clock().now())
        imu = self.imu_cache.getElemBeforeTime(self.get_clock().now())

        
        if frame is None or gps is None or imu is None:
            self.get_logger().warning(
                'Capture skipped: waiting for camera, GPS and IMU data. '
                'Send another trigger when all inputs are available.'
            )
            return

        message = TriggeredImageCapture()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame.header.frame_id
        message.image = frame
        message.location.lat = gps.latitude
        message.location.lon = gps.longitude
        message.location.alt = gps.altitude
        message.imu = imu
        self.publisher.publish(message)
        self.get_logger().info('Published triggered image with GPS and orientation')


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
