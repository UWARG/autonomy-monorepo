import logging
import math
from dataclasses import dataclass
import depthai as dai


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    label: int
    depth: float = math.nan

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)


class OakDPerception:
    """Builds an OAK-D pipeline with spatial object detection (DepthAI v3)."""

    def __init__(
        self,
        model_path: str,
        width: int = 640,
        height: int = 640,
        sensor_fps: float = 30.0,
        stereo_size: tuple[int, int] = (640, 400),
        confidence_threshold: float = 0.5,
        depth_lower_threshold: int = 100,
        depth_upper_threshold: int = 10000,
        bounding_box_scale_factor: float = 0.5,
    ) -> None:
        self._model_path = model_path
        self._width = width
        self._height = height
        self._sensor_fps = sensor_fps
        self._stereo_size = stereo_size
        self._confidence_threshold = confidence_threshold
        self._depth_lower_threshold = depth_lower_threshold
        self._depth_upper_threshold = depth_upper_threshold
        self._bounding_box_scale_factor = bounding_box_scale_factor

        self._pipeline = None
        self._device = None
        self._detection_queue = None

    def initialize_camera(self) -> bool:
        try:
            self._pipeline = dai.Pipeline()

            # Setup RGB Camera with requested output for NN
            cam_rgb = self._pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A, sensorFps = self._sensor_fps
            )
            nn_input = cam_rgb.requestOutput(
                (self._width, self._height), dai.ImgFrame.Type.BGR888p
            )

            # Setup Stereo Cameras & StereoDepth
            mono_left = self._pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B, sensorFps=self._sensor_fps
            )
            mono_right = self._pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C, sensorFps=self._sensor_fps
            )

            stereo = self._pipeline.create(dai.node.StereoDepth)
            mono_left.requestOutput(self._stereo_size).link(stereo.left)
            mono_right.requestOutput(self._stereo_size).link(stereo.right)

            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

            # Setup Spatial Detection Network
            model = dai.NNArchive(self._model_path)
            det = self._pipeline.create(dai.node.SpatialDetectionNetwork)
            
            nn_input.link(det.input)
            stereo.depth.link(det.inputDepth)
            
            det.setNNArchive(model)

            # These values are based on old model I believe
            det.setConfidenceThreshold(self._confidence_threshold)
            det.setDepthLowerThreshold(self._depth_lower_threshold)
            det.setDepthUpperThreshold(self._depth_upper_threshold)
            det.setBoundingBoxScaleFactor(self._bounding_box_scale_factor)

            # Output XLink Creation
            xout_det = self._pipeline.create(dai.node.XLinkOut)
            xout_det.setStreamName("detections")
            det.out.link(xout_det.input)

            # Device Execution & Queue Retrieval
            self._device = dai.Device(self._pipeline)
            self._detection_queue = self._device.getOutputQueue(
                name="detections", maxSize=4, blocking=False
            )

            logging.info("OAK-D perception pipeline initialized")
            return True

        except Exception as e:
            logging.error(f"OAK-D perception failed to initialize: {e}")
            return False

    def detect(self) -> list[Detection]:
        if self._detection_queue is None:
            logging.warning("OAK-D detection queue not initialized")
            return []

        results: list[Detection] = []
        try:
            in_det = self._detection_queue.tryGet()
            if in_det is None:
                return []

            for d in in_det.detections:
                depth_mm = d.spatialCoordinates.z
                depth = math.nan if depth_mm == 0 else float(depth_mm) / 1000.0

                results.append(
                    Detection(
                        x1=d.xmin * self._width,
                        y1=d.ymin * self._height,
                        x2=d.xmax * self._width,
                        y2=d.ymax * self._height,
                        confidence=d.confidence,
                        label=d.label,
                        depth=depth,
                    )
                )

        except Exception as e:
            logging.error(f"OAK-D detection read failed: {e}")
            return []

        return results

    def stop(self) -> None:
        if self._device is not None:
            self._device.close()
            self._device = None
            self._pipeline = None
            logging.info("OAK-D device stopped")