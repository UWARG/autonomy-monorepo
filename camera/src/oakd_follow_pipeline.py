"""Canonical DepthAI v2 pipeline for production follow and hardware logging."""

from __future__ import annotations


def build_follow_pipeline(
    blob_path: str,
    person_label: int = 15,
    camera_fps: int = 20,
    detector_stride: int = 1,
):
    """Build the one OAK-D pipeline used by production and calibration tools.

    ``detector_stride=1`` runs inference on every camera frame. A stride of two
    runs inference at half the camera rate while ``SHORT_TERM_IMAGELESS`` keeps
    emitting identity tracklets on the intervening frames.
    """
    if camera_fps <= 0:
        raise ValueError("camera_fps must be positive")
    if detector_stride not in (1, 2):
        raise ValueError("detector_stride must be 1 or 2")

    import depthai as dai

    pipeline = dai.Pipeline()
    color = pipeline.create(dai.node.ColorCamera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    detector = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
    tracker = pipeline.create(dai.node.ObjectTracker)
    tracklet_output = pipeline.create(dai.node.XLinkOut)
    detector_output = pipeline.create(dai.node.XLinkOut)

    color.setPreviewSize(640, 352)
    color.setInterleaved(False)
    color.setFps(camera_fps)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

    detector.setBlobPath(blob_path)
    detector.setConfidenceThreshold(0.5)
    detector.input.setBlocking(False)
    detector.setBoundingBoxScaleFactor(0.5)
    detector.setDepthLowerThreshold(300)
    detector.setDepthUpperThreshold(10000)

    tracker.setDetectionLabelsToTrack([person_label])
    tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
    if detector_stride == 1:
        tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        color.preview.link(detector.input)
        stereo.depth.link(detector.inputDepth)
        detector.passthrough.link(tracker.inputTrackerFrame)
    else:
        tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
        decimator = pipeline.create(dai.node.Script)
        decimator.setScript(
            f"""
while True:
    frame = node.io['frames'].get()
    depth = node.io['depth'].get()
    if frame.getSequenceNum() % {detector_stride} == 0:
        node.io['detections'].send(frame)
        node.io['detectionDepth'].send(depth)
"""
        )
        color.preview.link(decimator.inputs["frames"])
        stereo.depth.link(decimator.inputs["depth"])
        decimator.outputs["detections"].link(detector.input)
        decimator.outputs["detectionDepth"].link(detector.inputDepth)
        # Tracking runs at camera cadence even though detections do not.
        color.preview.link(tracker.inputTrackerFrame)

    tracklet_output.setStreamName("tracklets")
    detector_output.setStreamName("detector_frames")

    left.out.link(stereo.left)
    right.out.link(stereo.right)
    detector.passthrough.link(tracker.inputDetectionFrame)
    detector.out.link(tracker.inputDetections)
    tracker.out.link(tracklet_output.input)
    # This is emitted only when the tracker consumes a detector frame, making
    # host-side confirmed-vs-propagated classification explicit.
    tracker.passthroughDetectionFrame.link(detector_output.input)
    return pipeline
