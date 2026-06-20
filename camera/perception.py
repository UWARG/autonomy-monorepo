"""
OAK-D perception - adds on-device object detection to the OAK-D pipeline.

The base OakD (oakd.py) only does RGB + stereo depth capture. This module is
where you add a YoloSpatialDetectionNetwork node so the camera runs the .blob
model ON the device and returns bounding boxes with depth already fused.

Pipeline shape you are building (compare to the RGB/depth wiring in oakd.py):

    ColorCamera ---preview--> YoloSpatialDetectionNetwork ---> XLinkOut("detections")
    MonoLeft  --\                        ^
    MonoRight --/--> StereoDepth --depth--/   (depth feeds the spatial network)

The wiring pattern is identical to oakd.py: create node -> link inputs ->
create XLinkOut -> link output -> read from the output queue.

BLOCKER: you still need the .blob model file. Keep its path CONFIGURABLE
(constructor arg or constant) - never hardcode an absolute path.
"""

import logging
import math
from dataclasses import dataclass


@dataclass
class Detection:
    """A single detected target.

    Kept inside this file (instead of a shared detections.py) so the OAK-D
    perception work stays self-contained. The team can promote this to a
    shared type later, once the ArduCam side agrees on the same contract.

    Conventions to lock (ArduCam must match these later):
      - bbox is in PIXELS of the RGB frame, corner coords (x1, y1, x2, y2).
      - depth is metres; NaN when unknown.
    """

    x1: float
    y1: float
    x2: float
    y2: float

    confidence: float
    label: int  # class id from the model

    depth: float = math.nan

    @property
    def center(self) -> tuple[float, float]:
        """Pixel center (u, v) of the bbox. Handy for target localization."""
        # TODO(you): return the midpoint of the box.
        raise NotImplementedError


class OakDPerception:
    """Builds an OAK-D pipeline with spatial object detection."""

    # Must match the model's expected input size. YOLO blobs are commonly
    # 416x416 or 640x640 - check whatever the .blob was compiled for.
    WIDTH = 640
    HEIGHT = 640

    def __init__(self, blob_path: str) -> None:
        self._blob_path = blob_path
        self._pipeline = None
        self._device = None
        self._detection_queue = None

    def initialize_camera(self) -> bool:
        """Build the pipeline and start the device. Return True on success."""
        import depthai as dai  # noqa: F401  (imported lazily like in oakd.py)

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
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
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

            # 6. Create XLinkOut("detections") and link det.out -> xout.input.

            # 7. Start: self._device = dai.Device(self._pipeline)
            #    self._detection_queue = self._device.getOutputQueue(
            #        "detections", maxSize=4, blocking=False)

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
            # in_det = self._detection_queue.get()
            # for d in in_det.detections:
            #     DepthAI gives NORMALIZED corners: d.xmin, d.ymin, d.xmax, d.ymax
            #     in 0.0-1.0. Convert to pixels with self.WIDTH / self.HEIGHT.
            #
            #     Spatial coords are in mm on d.spatialCoordinates.{x,y,z};
            #     z is the depth -> divide by 1000 for metres (see
            #     sample_centre_depth in oakd.py for the same conversion).
            #
            #     results.append(Detection(
            #         x1=..., y1=..., x2=..., y2=...,
            #         confidence=d.confidence,
            #         label=d.label,
            #         depth=...,
            #     ))
            pass

        except Exception as e:
            logging.error(f"OAK-D detection read failed: {e}")
            return []

        return results

    def stop(self) -> None:
        """Release the device (mirror oakd.py.stop())."""
        # TODO(you): close self._device if it exists and set it back to None.
        pass
