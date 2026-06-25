from __future__ import annotations

import logging
import math

import numpy as np

from .abstract_camera import AbstractCamera
from .frame import CameraFrame

logger = logging.getLogger(__name__)

try:
    import depthai as dai
except ImportError:
    dai = None


class OakDCamera(AbstractCamera):
    """OAK-D RGB capture using the DepthAI v2 API."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
    ) -> None:
        super().__init__()
        if dai is None:
            raise ImportError(
                "depthai is required. Install with: apt install ros-humble-depthai."
            )
        self._width = width
        self._height = height
        self._fps = fps
        self._device: dai.Device | None = None
        self._queue: dai.DataOutputQueue | None = None

    def initialize_camera(self) -> bool:
        try:
            pipeline = self._build_pipeline()
            self._device = dai.Device(pipeline)
            self._queue = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
            return True
        except Exception:
            logger.exception("Failed to initialize OAK camera")
            self.stop()
            return False

    def capture_frame(self) -> CameraFrame | None:
        if self._queue is None:
            return None

        packet = self._queue.get()
        rgb = packet.getCvFrame()[:, :, ::-1].copy()  # BGR -> RGB

        return CameraFrame(rgb=rgb, depth=None, rgb_down=None, centre_depth=math.nan)

    def capture_rgb(self) -> np.ndarray | None:
        frame = self.capture_frame()
        return None if frame is None else frame.rgb

    def stop(self) -> None:
        self._queue = None
        if self._device is not None:
            self._device.close()
            self._device = None

    def _build_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(self._width, self._height)
        cam.setFps(self._fps)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)

        return pipeline
