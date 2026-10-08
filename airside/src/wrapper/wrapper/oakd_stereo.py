"""
OAK-D stereo node for rtabmap_sync/stereo_sync.

Publishes the rectified left/right mono images and their CameraInfo on the
topics stereo_sync subscribes to:
    left/image_rect, right/image_rect, left/camera_info, right/camera_info

OakDStereo implements the shared camera.src.abstract_camera.AbstractCamera
interface so it reuses its capture thread, retries and shutdown logic.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from camera.src.abstract_camera import AbstractCamera

# Mono sensors are 1280x800 (16:10); keep that aspect ratio.
OAKD_STEREO_DEFAULT_WIDTH = 640
OAKD_STEREO_DEFAULT_HEIGHT = 400
MAX_SYNC_ATTEMPTS = 10


@dataclass
class StereoFrame:
    """Rectified left/right mono8 images from one capture.

    timestamp is the capture time in epoch seconds, shared by both images.
    """

    left: np.ndarray
    right: np.ndarray
    timestamp: float
    sequence_num: int


class OakDStereo(AbstractCamera):
    """OAK-D rectified stereo pair.

    AbstractCamera is typed around CameraFrame, but capture_frame() and
    get_last_frame() return StereoFrame here.
    """

    def __init__(
        self,
        width: int = OAKD_STEREO_DEFAULT_WIDTH,
        height: int = OAKD_STEREO_DEFAULT_HEIGHT,
    ) -> None:
        super().__init__(width=width, height=height)
        self._pipeline = None
        self._left_queue = None
        self._right_queue = None
        self._intrinsics: np.ndarray | None = None
        self._baseline_m: float | None = None

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def intrinsics(self) -> np.ndarray | None:
        """3x3 camera matrix of the rectified images at the output resolution."""
        return self._intrinsics

    @property
    def baseline_m(self) -> float | None:
        """Distance between the left and right cameras, in metres."""
        return self._baseline_m

    def initialize_camera(self) -> bool:
        try:
            import depthai as dai

            self._pipeline = dai.Pipeline()

            left = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            right = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            # StereoDepth is only used here for on-device rectification;
            # RTAB-Map computes depth itself from the rectified pair.
            stereo = self._pipeline.create(dai.node.StereoDepth).build(
                left.requestOutput((self._width, self._height)),
                right.requestOutput((self._width, self._height)),
            )

            self._left_queue = stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=False)
            self._right_queue = stereo.rectifiedRight.createOutputQueue(maxSize=4, blocking=False)

            calib = self._pipeline.getDefaultDevice().readCalibration()
            # Rectification reprojects both images onto the right camera's
            # intrinsics (same convention as depthai-ros), with no distortion.
            self._intrinsics = np.array(
                calib.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_C, self._width, self._height
                )
            )
            # getBaselineDistance returns centimetres
            self._baseline_m = calib.getBaselineDistance(
                dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_B
            ) / 100.0

            self._pipeline.start()
            logging.info("OAK-D stereo camera initialized successfully")
            return True

        except Exception as e:
            logging.error(f"OAK-D stereo camera failed to initialize: {e}")
            return False

    def capture_frame(self) -> StereoFrame | None:
        if self._left_queue is None or self._right_queue is None:
            logging.warning("OAK-D stereo queues not initialized")
            return None

        try:
            import depthai as dai

            queues = [self._left_queue, self._right_queue]
            msgs = [q.get() for q in queues]

            # Left and right come from one capture only if their sequence numbers
            # match; otherwise advance the lagging stream until it catches up.
            for _ in range(MAX_SYNC_ATTEMPTS):
                seqs = [m.getSequenceNum() for m in msgs]
                newest = max(seqs)
                if min(seqs) == newest:
                    break
                for i, seq in enumerate(seqs):
                    if seq < newest:
                        msgs[i] = queues[i].get()
            else:
                logging.warning("OAK-D left/right streams could not be synchronised")
                return None

            left_msg, right_msg = msgs

            # getTimestamp() is on the host steady clock; convert to epoch seconds
            capture_age_s = (dai.Clock.now() - left_msg.getTimestamp()).total_seconds()

            return StereoFrame(
                left=left_msg.getCvFrame(),
                right=right_msg.getCvFrame(),
                timestamp=time.time() - capture_age_s,
                sequence_num=left_msg.getSequenceNum(),
            )

        except Exception as e:
            logging.error(f"OAK-D stereo frame capture failed: {e}")
            return None

    def close_camera(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            self._left_queue = None
            self._right_queue = None
            logging.info("OAK-D stereo pipeline stopped")


class OakDStereoNode(Node):
    PUBLISH_HZ = 30.0
    LEFT_FRAME_ID = "oak_left_camera_optical_frame"
    RIGHT_FRAME_ID = "oak_right_camera_optical_frame"

    def __init__(self) -> None:
        super().__init__("oakd_stereo_node")

        self._camera = OakDStereo()
        if not self._camera.start():
            raise RuntimeError("OAK-D stereo camera failed to start")

        self._left_image_pub = self.create_publisher(Image, "left/image_rect", 10)
        self._right_image_pub = self.create_publisher(Image, "right/image_rect", 10)
        self._left_info_pub = self.create_publisher(CameraInfo, "left/camera_info", 10)
        self._right_info_pub = self.create_publisher(CameraInfo, "right/camera_info", 10)

        self._left_info = self._make_camera_info(self.LEFT_FRAME_ID, baseline_m=0.0)
        self._right_info = self._make_camera_info(
            self.RIGHT_FRAME_ID, baseline_m=self._camera.baseline_m
        )
        self._last_sequence_num = -1
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)

        self.get_logger().info(
            f"OAK-D stereo node ready - {self._camera.width}x{self._camera.height}, "
            f"baseline {self._camera.baseline_m:.4f} m"
        )

    def _make_camera_info(self, frame_id: str, baseline_m: float) -> CameraInfo:
        """CameraInfo for a rectified image; the right camera carries the baseline in P."""
        k = self._camera.intrinsics
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]

        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = self._camera.width
        info.height = self._camera.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Tx = -fx * baseline (0 for the left camera), per ROS stereo convention
        info.p = [fx, 0.0, cx, -fx * baseline_m, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _make_image(self, img: np.ndarray, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "mono8"
        msg.step = img.shape[1]
        msg.data = img.tobytes()
        return msg

    def _publish_frame(self) -> None:
        frame = self._camera.get_last_frame()
        if frame is None or frame.sequence_num == self._last_sequence_num:
            return
        self._last_sequence_num = frame.sequence_num

        # Both images and both infos share the capture stamp so stereo_sync can
        # use exact synchronisation
        stamp = Time(nanoseconds=int(frame.timestamp * 1e9)).to_msg()
        self._left_info.header.stamp = stamp
        self._right_info.header.stamp = stamp

        self._left_image_pub.publish(self._make_image(frame.left, self.LEFT_FRAME_ID, stamp))
        self._right_image_pub.publish(self._make_image(frame.right, self.RIGHT_FRAME_ID, stamp))
        self._left_info_pub.publish(self._left_info)
        self._right_info_pub.publish(self._right_info)

    def destroy_node(self) -> None:
        self._camera.stop()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OakDStereoNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
