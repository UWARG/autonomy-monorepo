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
