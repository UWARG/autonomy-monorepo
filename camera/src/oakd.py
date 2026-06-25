from __future__ import annotations
import math
import numpy as np
from .abstract_camera import AbstractCamera
from .frame import CameraFrame
try:
    import depthai as dai
except ImportError:  # allows importing the module without hardware SDK installed
    dai = None
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
class OakDCamera(AbstractCamera):
  def __init__(
      self,
      rgb_resolution: dai.ColorCameraProperties.SensorResolution | None = None,
      fps: float = 30.0,
      preview_width: int = 640,
      preview_height: int = 480,
  ) -> None:
      super().__init__()
      if dai is None:
          raise ImportError("depthai is required for OakDCamera. Install with: pip install depthai")
      self._rgb_resolution = rgb_resolution or dai.ColorCameraProperties.SensorResolution.THE_1080_P
      self._fps = fps
      self._preview_width = preview_width
      self._preview_height = preview_height
      self._device: dai.Device | None = None
      self._rgb_queue: dai.DataOutputQueue | None = None
      self._depth_queue: dai.DataOutputQueue | None = None
  def initialize_camera(self) -> bool:
      try:
          pipeline = self._build_pipeline()
          self._device = dai.Device(pipeline)
          self._rgb_queue = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
          self._depth_queue = self._device.getOutputQueue("depth", maxSize=4, blocking=False)
          return True
      except Exception as e:
          logger.error(f"Failed to initialize camera: {e}")
          self.stop()
          return False
  def capture_frame(self) -> CameraFrame | None:
      if self._rgb_queue is None:
          return None
      rgb_packet = self._rgb_queue.tryGet()
      if rgb_packet is None:
          return None
      rgb = rgb_packet.getCvFrame()  # BGR; convert if you want RGB everywhere
      rgb = rgb[:, :, ::-1].copy()   # BGR -> RGB
      depth = None
      centre_depth = math.nan
      if self._depth_queue is not None:
          depth_packet = self._depth_queue.tryGet()
          if depth_packet is not None:
              depth = depth_packet.getFrame()  # uint16, millimeters
              h, w = depth.shape
              centre_depth = float(depth[h // 2, w // 2]) / 1000.0  # mm -> m
      return CameraFrame(
          rgb=rgb,
          depth=depth,
          rgb_down=None,
          centre_depth=centre_depth,
      )
  def stop(self) -> None:
      self._rgb_queue = None
      self._depth_queue = None
      if self._device is not None:
          self._device.close()
          self._device = None
  def _build_pipeline(self) -> dai.Pipeline:
      pipeline = dai.Pipeline()
      cam_rgb = pipeline.create(dai.node.ColorCamera)
      cam_rgb.setResolution(self._rgb_resolution)
      cam_rgb.setFps(self._fps)
      cam_rgb.setPreviewSize(self._preview_width, self._preview_height)
      cam_rgb.setInterleaved(False)
      cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
      mono_left = pipeline.create(dai.node.MonoCamera)
      mono_right = pipeline.create(dai.node.MonoCamera)
      mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
      mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
      mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
      mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
      stereo = pipeline.create(dai.node.StereoDepth)
      stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
      mono_left.out.link(stereo.left)
      mono_right.out.link(stereo.right)
      xout_rgb = pipeline.create(dai.node.XLinkOut)
      xout_rgb.setStreamName("rgb")
      cam_rgb.preview.link(xout_rgb.input)
      xout_depth = pipeline.create(dai.node.XLinkOut)
      xout_depth.setStreamName("depth")
      stereo.depth.link(xout_depth.input)
      return pipeline