#!/usr/bin/env python3
"""Publish a still image through apriltag_ros and print detections / TF.

Assumes apriltag_node is running with the usual remappings:
  image_rect  -> camera/image
  camera_info -> camera/camera_info

Example (with the engine stack already up, or just apriltag_node):
  ros2 run --prefix '' python3 dummy_apriltag_feed.py
  # or from airside_prec/:
  python3 dummy_apriltag_feed.py
"""

from __future__ import annotations

import os
import sys

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None  # type: ignore[misc, assignment]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IMAGE = os.path.join(SCRIPT_DIR, "images", "output.jpg")
DEFAULT_CAMERA_INFO = os.path.join(SCRIPT_DIR, "src", "camera_info.yaml")

# Match camera_node / apriltag remappings in engine.launch.py
IMAGE_TOPIC = "camera/image"
CAMERA_INFO_TOPIC = "camera/camera_info"
TF_TOPIC = "/tf"
DETECTIONS_TOPIC = "detections"

PUBLISH_HZ = 5.0


class DummyApriltagFeed(Node):
    def __init__(self, image_path: str, camera_info_path: str) -> None:
        super().__init__("dummy_apriltag_feed")

        frame = cv2.imread(image_path)
        if frame is None:
            raise FileNotFoundError(f"Failed to load image: {image_path}")

        self._bridge = CvBridge()
        self._frame = frame
        self._height, self._width = frame.shape[:2]
        self._camera_info = self._load_camera_info(camera_info_path)
        self._got_result = False

        self._image_pub = self.create_publisher(Image, IMAGE_TOPIC, 10)
        self._info_pub = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, 10)
        self.create_subscription(TFMessage, TF_TOPIC, self._tf_callback, 10)
        if AprilTagDetectionArray is not None:
            self.create_subscription(
                AprilTagDetectionArray,
                DETECTIONS_TOPIC,
                self._detections_callback,
                10,
            )

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f"Publishing {image_path} ({self._width}x{self._height}) on "
            f"'{IMAGE_TOPIC}' @ {PUBLISH_HZ} Hz; waiting for /tf"
            + (" and detections" if AprilTagDetectionArray is not None else "")
        )

    def _load_camera_info(self, path: str) -> CameraInfo:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        calib_w = float(data["width"])
        calib_h = float(data["height"])
        sx = self._width / calib_w
        sy = self._height / calib_h

        k = list(data["camera_matrix"]["data"])
        # Scale fx, cx, fy, cy for the still-image resolution.
        k[0] *= sx
        k[2] *= sx
        k[4] *= sy
        k[5] *= sy

        p = list(data["projection_matrix"]["data"])
        p[0] *= sx
        p[2] *= sx
        p[5] *= sy
        p[6] *= sy

        msg = CameraInfo()
        msg.height = self._height
        msg.width = self._width
        msg.distortion_model = data["distortion_model"]
        msg.k = k
        msg.d = list(data["distortion_coefficients"]["data"])
        msg.r = list(data["rectification_matrix"]["data"])
        msg.p = p
        return msg

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()

        img_msg = self._bridge.cv2_to_imgmsg(self._frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "camera"

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "camera"
        info.height = self._camera_info.height
        info.width = self._camera_info.width
        info.distortion_model = self._camera_info.distortion_model
        info.k = self._camera_info.k
        info.d = self._camera_info.d
        info.r = self._camera_info.r
        info.p = self._camera_info.p

        self._info_pub.publish(info)
        self._image_pub.publish(img_msg)

    def _tf_callback(self, msg: TFMessage) -> None:
        if not msg.transforms:
            self.get_logger().info("TF: (empty)")
            return

        for t in msg.transforms:
            tr = t.transform.translation
            rot = t.transform.rotation
            self.get_logger().info(
                f"TF {t.child_frame_id} <- {t.header.frame_id}: "
                f"t=({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f}) "
                f"q=({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})"
            )
        self._got_result = True

    def _detections_callback(self, msg: AprilTagDetectionArray) -> None:
        if not msg.detections:
            self.get_logger().info("detections: (none)")
            return
        for det in msg.detections:
            self.get_logger().info(
                f"detection id={det.id} ham={det.hamming} "
                f"centre=({det.centre.x:.1f}, {det.centre.y:.1f})"
            )
        self._got_result = True


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    image_path = argv[0] if argv else DEFAULT_IMAGE
    camera_info_path = argv[1] if len(argv) > 1 else DEFAULT_CAMERA_INFO

    rclpy.init()
    node = DummyApriltagFeed(image_path, camera_info_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._got_result:
            node.get_logger().info("Got at least one apriltag result.")
        else:
            node.get_logger().warn(
                "No TF/detections received. Is apriltag_node running "
                "with remaps to camera/image and camera/camera_info?"
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
