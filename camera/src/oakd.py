"""
OakD - OAK-D camera implementation of AbstractCamera
"""

import logging
import math
from pathlib import Path
from time import monotonic, sleep

import numpy as np

from .abstract_camera import AbstractCamera
from .constants import OAKD_DEFAULT_HEIGHT, OAKD_DEFAULT_WIDTH
from .frame import CameraFrame

class OakD(AbstractCamera):

    def __init__(
        self,
        width: int = OAKD_DEFAULT_WIDTH,
        height: int = OAKD_DEFAULT_HEIGHT,
        *,
        enable_point_cloud: bool = False,
    ) -> None:
        super().__init__(width=width, height=height)
        self._pipeline = None
        self._enable_point_cloud = enable_point_cloud
        self._point_cloud_queue = None
        self._pipeline_started_at = 0.0
        self._rgb_queue = None
        self._depth_queue = None

    def initialize_camera(self) -> bool:
        try:
            import depthai as dai

            self.close_camera()
            self._pipeline = dai.Pipeline()

            # RGB stream
            cam = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            self._rgb_queue = cam.requestOutput(
                (self._width, self._height),
                dai.ImgFrame.Type.BGR888p
            ).createOutputQueue(maxSize=4, blocking=False)

            # Stereo depth stream
            left = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            right = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = self._pipeline.create(dai.node.StereoDepth).build(
                left.requestOutput((1280, 800)), 
                right.requestOutput((1280, 800))
            )
            if self._enable_point_cloud:
                color = cam.requestOutput(
                    (self._width, self._height),
                    dai.ImgFrame.Type.RGB888i,
                    enableUndistortion=True,
                )
                cloud = self._pipeline.create(dai.node.PointCloud)
                cloud.initialConfig.setLengthUnit(dai.LengthUnit.METER)
                if self._pipeline.getDefaultDevice().getPlatform() == dai.Platform.RVC4:
                    align = self._pipeline.create(dai.node.ImageAlign)
                    stereo.depth.link(align.input)
                    color.link(align.inputAlignTo)
                    align.outputAligned.link(cloud.inputDepth)
                else:
                    color.link(stereo.inputAlignTo)
                    stereo.depth.link(cloud.inputDepth)
                color.link(cloud.inputColor)
                self._point_cloud_queue = cloud.outputPointCloud.createOutputQueue(
                    maxSize=1, blocking=False
                )

            self._depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

            self._pipeline.start()
            self._pipeline_started_at = monotonic()
            logging.info("OAK-D camera initialized successfully")
            return True

        except Exception as e:
            logging.error(f"OAK-D camera failed to initialize: {e}")
            self.close_camera()
            return False

    def capture_frame(self) -> CameraFrame | None:
        if self._rgb_queue is None or self._depth_queue is None:
            logging.warning("OAK-D queues not initialized")
            return None

        try:
            rgb_frame = self._rgb_queue.get().getCvFrame()
            depth_frame = self._depth_queue.get().getCvFrame()
            centre_depth = self.sample_centre_depth(depth_frame)

            return CameraFrame(
                rgb=rgb_frame,
                depth=depth_frame,
                centre_depth=centre_depth,
            )

        except Exception as e:
            logging.error(f"OAK-D frame capture failed: {e}")
            return None

    def capture_point_cloud(
        self, timeout: float = 10.0, warmup: float = 2.0
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return valid XYZ points in meters and matching uint8 RGB colors.

        Requires initialization with enable_point_cloud=True. Warm-up is measured
        from pipeline startup. Timeout begins after any remaining warm-up.
        """
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and > 0")
        if not math.isfinite(warmup) or warmup < 0:
            raise ValueError("warmup must be finite and >= 0")
        pipeline = self._pipeline
        queue = self._point_cloud_queue
        if pipeline is None or queue is None:
            raise RuntimeError(
                "Initialize OakD with enable_point_cloud=True before capturing."
            )

        settled = self._pipeline_started_at + warmup
        deadline = max(monotonic(), settled) + timeout
        while self._pipeline is pipeline and pipeline.isRunning() and monotonic() < deadline:
            message = queue.tryGet()
            if message is None or monotonic() < settled:
                sleep(0.01)
                continue
            xyz, rgba = message.getPointsRGB()
            valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
            if valid.any():
                return xyz[valid], rgba[valid, :3]

        raise RuntimeError("No valid OAK-D point cloud received before capture ended.")

    def save_point_cloud(
        self, path: str | Path, timeout: float = 10.0, warmup: float = 2.0
    ) -> None:
        """Capture and export a binary PLY with XYZ in meters and RGB colors."""
        from .oakd_point_cloud import write_ply

        xyz, rgb = self.capture_point_cloud(timeout=timeout, warmup=warmup)
        write_ply(Path(path), xyz, rgb)

    def sample_centre_depth(self, depth_frame: np.ndarray) -> float:
        if depth_frame is None:
            return math.nan

        cy, cx = depth_frame.shape[0] // 2, depth_frame.shape[1] // 2
        region = depth_frame[cy - 2 : cy + 3, cx - 2 : cx + 3]
        valid = region[region > 0]

        if len(valid) == 0:
            return math.nan

        return float(np.median(valid)) / 1000.0

    def close_camera(self) -> None:
        pipeline = self._pipeline
        self._pipeline = None
        self._rgb_queue = None
        self._depth_queue = None
        self._point_cloud_queue = None
        self._pipeline_started_at = 0.0
        if pipeline is not None:
            try:
                pipeline.stop()
                logging.info("OAK-D pipeline stopped")
            except Exception as error:
                logging.warning("OAK-D pipeline shutdown failed: %s", error)
