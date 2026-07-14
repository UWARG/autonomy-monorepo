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

    def __init__(
        self,
        blob_path: str,
        width: int = 640,
        height: int = 640,
        confidence_threshold: float = 0.5,
        num_classes: int = 4,
        coordinate_size: int = 1,
        anchors: list[float] | None = None,
        anchor_masks: dict | None = None,
        depth_lower_threshold: int = 100,
        depth_upper_threshold: int = 10000,
        bounding_box_scale_factor: float = 0.5,
    ) -> None:
        self._blob_path = blob_path
        self._width = width
        self._height = height
        self._confidence_threshold = confidence_threshold
        self._num_classes = num_classes
        self._coordinate_size = coordinate_size
        self._anchors = anchors if anchors is not None else []
        self._anchor_masks = anchor_masks if anchor_masks is not None else {}
        self._depth_lower_threshold = depth_lower_threshold
        self._depth_upper_threshold = depth_upper_threshold
        self._bounding_box_scale_factor = bounding_box_scale_factor

        self._pipeline = None
        self._device = None
        self._detection_queue = None

    def initialize_camera(self) -> bool:
        import depthai as dai

        try:
            self._pipeline = dai.Pipeline()

            # RGB stream
            cam_rgb = self._pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(self._width, self._height)
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

            det.setConfidenceThreshold(self._confidence_threshold)

            # defaults are placeholders until old model's values found
            det.setNumClasses(self._num_classes)
            det.setCoordinateSize(self._coordinate_size)
            det.setAnchors(self._anchors)
            det.setAnchorMasks(self._anchor_masks)

            det.setDepthLowerThreshold(self._depth_lower_threshold)
            det.setDepthUpperThreshold(self._depth_upper_threshold)
            det.setBoundingBoxScaleFactor(self._bounding_box_scale_factor)

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
            logging.info("OAK-D device stopped")
