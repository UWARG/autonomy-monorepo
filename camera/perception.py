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

from detections import Detection


class OakDPerception:
    """Builds an OAK-D pipeline with spatial object detection."""

    # Must match the model's expected input size. YOLO blobs are commonly
    # 416x416 or 640x640 - check whatever the .blob was compiled for.
    WIDTH = 640
    HEIGHT = 640

    def __init__(self, blob_path: str) -> None:
        # TODO(you): store blob_path. Also init the same handles oakd.py keeps:
        #   self._pipeline, self._device, and a detection queue handle.
        self._blob_path = blob_path
        self._pipeline = None
        self._device = None
        self._detection_queue = None

    def initialize_camera(self) -> bool:
        """Build the pipeline and start the device. Return True on success."""
        import depthai as dai  # noqa: F401  (imported lazily like in oakd.py)

        try:
            # 1. Create the pipeline (dai.Pipeline()).

            # 2. Create the RGB ColorCamera, set preview size to WIDTH/HEIGHT,
            #    setInterleaved(False). The spatial detection network wants
            #    BGR planar input - check the DepthAI docs for the exact
            #    color order / interleaving the YoloSpatialDetectionNetwork needs.

            # 3. Recreate the stereo depth half from oakd.py (mono left + mono
            #    right + StereoDepth). The spatial network needs the depth map,
            #    so set stereo.setDepthAlign(dai.CameraBoardSocket.RGB) so depth
            #    lines up with the RGB frame the boxes are measured in.

            # 4. Create the detection node:
            #       det = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
            #       det.setBlobPath(self._blob_path)
            #       det.setConfidenceThreshold(...)
            #    Then set the YOLO-specific decoding params that MUST match how
            #    the model was trained/exported:
            #       setNumClasses, setCoordinateSize, setAnchors,
            #       setAnchorMasks, setIouThreshold
            #    And the spatial params:
            #       setDepthLowerThreshold / setDepthUpperThreshold (mm),
            #       setBoundingBoxScaleFactor

            # 5. Link inputs:
            #       cam_rgb.preview -> det.input
            #       stereo.depth    -> det.inputDepth

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
