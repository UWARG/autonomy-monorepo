"""
OakD - OAK-D camera implementation of AbstractCamera
"""

import logging
import math
import numpy as np

from abstract_camera import AbstractCamera
from frame import CameraFrame


class OakD(AbstractCamera):

    WIDTH = 640
    HEIGHT = 480

    def __init__(self) -> None:
        super().__init__()
        self._pipeline = None
        self._rgb_queue = None
        self._depth_queue = None

    def initialize_camera(self) -> bool:
        import depthai as dai

        try:
            self._pipeline = dai.Pipeline()

            # RGB stream
            cam = self._pipeline.create(dai.node.Camera).build()
            self._rgb_queue = cam.requestOutput(
                (self.WIDTH, self.HEIGHT), dai.ImgFrame.Type.BGR888p
            ).createOutputQueue(maxSize=4, blocking=False)

            # Stereo depth stream
            left = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            right = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = self._pipeline.create(dai.node.StereoDepth).build(
                left.requestOutput((1280, 800)), right.requestOutput((1280, 800))
            )
            self._depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

            self._pipeline.start()
            logging.info("OAK-D camera initialized successfully")
            return True

        except Exception as e:
            logging.error(f"OAK-D camera failed to initialize: {e}")
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
                rgb_down=None,
                centre_depth=centre_depth,
            )

        except Exception as e:
            logging.error(f"OAK-D frame capture failed: {e}")
            return None

    def sample_centre_depth(self, depth_frame: np.ndarray) -> float:
        if depth_frame is None:
            return math.nan

        cy, cx = depth_frame.shape[0] // 2, depth_frame.shape[1] // 2

        # sample a 5x5 region around the centre and take median of non-zero values
        region = depth_frame[cy-2:cy+3, cx-2:cx+3]
        valid = region[region > 0]

        if len(valid) == 0:
            return math.nan

        return float(np.median(valid)) / 1000.0

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            logging.info("OAK-D pipeline stopped")