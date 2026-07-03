"""
OAK-D perception - adds on-device object detection to the OAK-D pipeline.
"""

import logging
import math
from dataclasses import dataclass


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float

    confidence: float
    label: int  # class id from the model

    depth: float = math.nan

    @property
    def center(self) -> tuple[float, float]:
        x_center = (self.x1 + self.x2) / 2
        y_center = (self.y1 + self.y2) / 2
        return [x_center, y_center]


class OakDPerception:
    """Builds an OAK-D pipeline with spatial object detection."""

    WIDTH = 640
    HEIGHT = 640

    def __init__(self, blob_path: str) -> None:
        self._blob_path = blob_path
        self._pipeline = None
        self._device = None
        self._detection_queue = None

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
            stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            mono_right.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_400_P
            )
            mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            # Detection node
            det = self._pipeline.create(dai.node.YoloSpatialDetectionNetwork)
            det.setBlobPath(self._blob_path)

            # Set to 0.5 for now
            det.setConfidenceThreshold(0.5)

            # Waiting for test model to set these params
            det.setNumClasses(4)
            det.setCoordinateSize(1)
            det.setAnchors([])
            det.setAnchorMasks({})

            # Set to random values for now
            det.setDepthLowerThreshold(100)
            det.setDepthUpperThreshold(10000)
            det.setBoundingBoxScaleFactor(0.5)

            cam_rgb.preview.link(det.input)
            stereo.depth.link(det.inputDepth)

            xout_det = self._pipeline.create(dai.node.XLinkOut)
            xout_det.setStreamName("detections")
            det.out.link(xout_det.input)

            self._device = dai.Device(self._pipeline)
            self._detection_queue = self._device.getOutputQueue(
                "detections", maxsize=4, blocking=False
            )

            logging.info("OAK-D perception pipeline initialized")
            return True

        except Exception as e:
            logging.error(f"OAK-D perception failed to initialize: {e}")
            return False

    def detect(self) -> list[Detection]:
        """Read the latest detections and return them in pixel coords.

        Same-signature sibling of whatever ArduCam will expose, so downstream
        code is camera-agnostic.
        """
        if self._detection_queue is None:
            logging.warning("OAK-D detection queue not initialized")
            return []

        results: list[Detection] = []
        try:
            in_det = self._detection_queue.get()
            for d in in_det.detections:
                depth_mm = d.spatialCoordinates.z
                depth = math.nan if depth_mm == 0 else float(depth_mm) / 1000.0

                results.append(
                    Detection(
                        x1=d.xmin * self.WIDTH,
                        y1=d.ymin * self.HEIGHT,
                        x2=d.xmax * self.WIDTH,
                        y2=d.ymax * self.HEIGHT,
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
            logging.info("OAK-D device stopped")
