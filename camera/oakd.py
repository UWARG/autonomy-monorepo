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
        self._device = None
        self._rgb_queue = None
        self._depth_queue = None

    def initialize_camera(self) -> bool:
        import depthai as dai

        try:
            self._pipeline = dai.Pipeline()

            # RGB stream
            cam_rgb = self._pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(self.WIDTH, self.HEIGHT)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

            xout_rgb = self._pipeline.create(dai.node.XLinkOut)
            xout_rgb.setStreamName("rgb")
            cam_rgb.preview.link(xout_rgb.input)

            # Stereo depth stream
            mono_left = self._pipeline.create(dai.node.MonoCamera)
            mono_right = self._pipeline.create(dai.node.MonoCamera)
            stereo = self._pipeline.create(dai.node.StereoDepth)

            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            xout_depth = self._pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName("depth")
            stereo.depth.link(xout_depth.input)

            # Start pipeline
            self._device = dai.Device(self._pipeline)
            self._rgb_queue = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self._depth_queue = self._device.getOutputQueue("depth", maxSize=4, blocking=False)

            logging.info("OAK-D camera initialized successfully")
            return True
        
        except Exception as e:
            logging.error(f"OAK-D camera failed to initialize: {e}")
            return False

    def capture_frame(self) -> CameraFrame | None:
        # initialization check
        if self._rgb_queue is None or self._depth_queue is None:
            logging.warning("OAK-D queues not initialized")
            return None

        try:
            rgb_frame = self._rgb_queue.get().getCvFrame()
            depth_frame = self._depth_queue.get().getFrame()
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
        depth_value = depth_frame[cy, cx]

        # convert to meters since OAK-D returns depth in millimeters
        if depth_value == 0:
            return math.nan
    
        return float(depth_value) / 1000.0

    def stop(self) -> None:
        if self._device is not None:
            self._device.close()
            self._device = None
            logging.info("OAK-D device stopped")


