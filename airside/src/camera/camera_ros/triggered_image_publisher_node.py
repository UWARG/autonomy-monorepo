"""Serve the latest image, GPS location and IMU on a capture request."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix
import message_filters

from airside_interfaces.srv import CaptureImage

JPEG_QUALITY = 90


def to_jpeg(frame: Image) -> CompressedImage | None:
    """JPEG-encode an rgb8/bgr8 frame, keeping its header; None for other encodings."""
    if frame.encoding not in ('rgb8', 'bgr8'):
        return None
    pixels = np.frombuffer(bytes(frame.data), np.uint8).reshape(frame.height, frame.step)
    pixels = pixels[:, :frame.width * 3].reshape(frame.height, frame.width, 3)
    if frame.encoding == 'rgb8':
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    ok, jpeg = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        return None
    compressed = CompressedImage()
    compressed.header = frame.header
    compressed.format = 'jpeg'
    compressed.data = jpeg.tobytes()
    return compressed


class TriggeredImagePublisherNode(Node):
    """Cache sensor messages and return one complete bundle per capture request."""

    def __init__(self):
        super().__init__('triggered_image_publisher')

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



        self.service = self.create_service(
            CaptureImage, '/capture_image', self.capture_callback
        )
        self.get_logger().info('Ready for capture requests on /capture_image')


    def capture_callback(self, request: CaptureImage.Request, response: CaptureImage.Response):
        """Return cached data without altering the source image or its timestamp."""

        frame = self.image_cache.getElemBeforeTime(self.get_clock().now())
        gps = self.gps_cache.getElemBeforeTime(self.get_clock().now())
        imu = self.imu_cache.getElemBeforeTime(self.get_clock().now())

        missing = [
            name for name, value in (('camera', frame), ('GPS', gps), ('IMU', imu))
            if value is None
        ]
        if missing:
            response.success = False
            response.message = f'Waiting for {", ".join(missing)} data'
            self.get_logger().warning(f'Capture skipped: {response.message}')
            return response

        jpeg = to_jpeg(frame)
        if jpeg is None:
            response.success = False
            response.message = f'Cannot JPEG-encode camera encoding {frame.encoding}'
            self.get_logger().warning(f'Capture skipped: {response.message}')
            return response

        response.success = True
        response.header.stamp = self.get_clock().now().to_msg()
        response.header.frame_id = frame.header.frame_id
        response.image = jpeg
        response.location.lat = gps.latitude
        response.location.lon = gps.longitude
        response.location.alt = gps.altitude
        response.imu = imu
        self.get_logger().info('Served capture with GPS and orientation')
        return response


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
