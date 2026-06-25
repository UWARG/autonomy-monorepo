from __future__ import annotations
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
      self
  ) -> None:
      if dai is None:
        raise ImportError("depthai is required for OakDCamera. Install with: pip install depthai")
      super().__init__()
      self._video_queue: dai.DataOutputQueue | None = None
      self._pipeline: dai.Pipeline | None = None

  def initialize_camera(self) -> bool:
      try:
          self._pipeline = self._build_pipeline()
          self._pipeline.start()
          return True
      except Exception as e:
          logger.error(f"Failed to initialize camera: {e}")
          self.stop()
          return False
  def capture_frame(self) -> CameraFrame | None:
      if self._video_queue is None:
        return None
      image_frame = self._video_queue.get()
      bgr = image_frame.getCvFrame()
      rgb = bgr[:, :, ::-1].copy()
      return CameraFrame(
          rgb=rgb,
          depth=None,
          rgb_down=None,
      )

  def stop(self) -> None:
    if self._pipeline is not None:
      self._pipeline.stop()
      self._pipeline = None
      self._video_queue = None

  def _build_pipeline(self) -> dai.Pipeline:
    try:
      pipeline = dai.Pipeline()
      cam=pipeline.create(dai.node.Camera).build()
      video_out = cam.requestOutput(1920, 1080)
      self._video_queue = video_out.createOutputQueue(maxSize=4, blocking=False)
      return pipeline
    except Exception as e:
      logger.error(f"Failed to build pipeline: {e}")
      return None