"""
OAK-D camera with optional Basalt visual-inertial odometry.
"""

import logging
import math
import time

import numpy as np

from .abstract_camera import AbstractCamera
from .constants import OAKD_DEFAULT_HEIGHT, OAKD_DEFAULT_WIDTH
from .frame import CameraFrame
from .vio import VioPose


VIO_QUEUE_SIZE = 8


class OakD(AbstractCamera):
    def __init__(
        self,
        width: int = OAKD_DEFAULT_WIDTH,
        height: int = OAKD_DEFAULT_HEIGHT,
        *,
        enable_vio: bool = False,
    ) -> None:
        super().__init__(width=width, height=height)
        self._enable_vio = enable_vio
        self._pipeline = None
        self._rgb_queue = None
        self._depth_queue = None
        self._vio_queue = None

    def initialize_camera(self) -> bool:
        # Release resources before starting a new pipeline.
        self.close_camera()

        try:
            import depthai as dai

            if self._enable_vio and not hasattr(dai.node, "BasaltVIO"):
                raise RuntimeError(
                    "This DepthAI build does not include BasaltVIO."
                )

            self._pipeline = dai.Pipeline()

            # RGB stream.
            cam = self._pipeline.create(dai.node.Camera).build()
            self._rgb_queue = cam.requestOutput(
                (self._width, self._height),
                dai.ImgFrame.Type.BGR888p,
            ).createOutputQueue(maxSize=4, blocking=False)

            # Share stereo images between depth processing and VIO.
            if self._enable_vio:
                fps = 60
                image_size = (640, 400)
            else:
                fps = 30
                image_size = (1280, 800)

            left = self._pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B,
                sensorFps=fps,
            )
            right = self._pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C,
                sensorFps=fps,
            )

            left_out = left.requestOutput(image_size)
            right_out = right.requestOutput(image_size)

            stereo = self._pipeline.create(dai.node.StereoDepth).build(
                left_out,
                right_out,
            )
            self._depth_queue = stereo.depth.createOutputQueue(
                maxSize=4,
                blocking=False,
            )

            if self._enable_vio:
                imu = self._pipeline.create(dai.node.IMU)
                imu.enableIMUSensor(
                    [
                        dai.IMUSensor.ACCELEROMETER_RAW,
                        dai.IMUSensor.GYROSCOPE_RAW,
                    ],
                    200,
                )
                imu.setBatchReportThreshold(1)
                imu.setMaxBatchReports(10)

                vio = self._pipeline.create(dai.node.BasaltVIO)
                vio.setImuUpdateRate(200)

                left_out.link(vio.left)
                right_out.link(vio.right)
                imu.out.link(vio.imu)

                self._vio_queue = vio.transform.createOutputQueue(
                    maxSize=VIO_QUEUE_SIZE,
                    blocking=False,
                )

            self._pipeline.start()
            logging.info(
                "OAK-D camera initialized successfully (VIO=%s)",
                self._enable_vio,
            )
            return True

        except Exception:
            logging.exception("OAK-D camera failed to initialize")
            self.close_camera()
            return False

    def capture_pose(self) -> VioPose | None:
        """Return the newest available valid pose without waiting.

        Intermediate queued estimates are discarded. Position and
        orientation remain in the SDK's local coordinate frame.
        """
        if self._vio_queue is None:
            return None

        newest = None

        for _ in range(VIO_QUEUE_SIZE):
            try:
                message = self._vio_queue.tryGet()
                if message is None:
                    break

                received = time.monotonic()
                translation = message.getTranslation()
                quaternion = message.getQuaternion()

                position = (
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                )
                rotation = (
                    float(quaternion.qx),
                    float(quaternion.qy),
                    float(quaternion.qz),
                    float(quaternion.qw),
                )

                if not all(
                    math.isfinite(value)
                    for value in (*position, *rotation)
                ):
                    raise ValueError("Pose contains a non-finite value")

                norm = math.hypot(*rotation)
                if norm < 1e-12 or not math.isfinite(norm):
                    raise ValueError("Pose contains an invalid quaternion")

                timestamp = message.getTimestamp().total_seconds()

                newest = VioPose(
                    position_m=position,
                    orientation_xyzw=tuple(
                        value / norm for value in rotation
                    ),
                    timestamp_s=(
                        timestamp
                        if math.isfinite(timestamp) and timestamp > 0
                        else None
                    ),
                    received_monotonic_s=received,
                )

            except (AttributeError, TypeError, ValueError, OverflowError):
                logging.warning(
                    "Discarding invalid VIO pose",
                    exc_info=True,
                )
            except Exception:
                logging.exception("Failed to read VIO queue")
                break

        return newest

    def capture_frame(self) -> CameraFrame | None:
        if self._rgb_queue is None or self._depth_queue is None:
            logging.warning("OAK-D queues not initialized")
            return None

        try:
            rgb_frame = self._rgb_queue.get().getCvFrame()
            depth_frame = self._depth_queue.get().getCvFrame()

            return CameraFrame(
                rgb=rgb_frame,
                depth=depth_frame,
                centre_depth=self.sample_centre_depth(depth_frame),
            )

        except Exception:
            logging.exception("OAK-D frame capture failed")
            return None

    def sample_centre_depth(self, depth_frame: np.ndarray) -> float:
        if depth_frame is None:
            return math.nan

        cy = depth_frame.shape[0] // 2
        cx = depth_frame.shape[1] // 2
        region = depth_frame[
            max(0, cy - 2) : cy + 3,
            max(0, cx - 2) : cx + 3,
        ]
        valid = region[region > 0]

        if valid.size == 0:
            return math.nan

        return float(np.median(valid)) / 1000.0

    def close_camera(self) -> None:
        pipeline = self._pipeline

        self._pipeline = None
        self._rgb_queue = None
        self._depth_queue = None
        self._vio_queue = None

        if pipeline is not None:
            try:
                pipeline.stop()
                logging.info("OAK-D pipeline stopped")
            except Exception:
                logging.exception("Failed to stop OAK-D pipeline")