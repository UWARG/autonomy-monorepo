"""
Target tracking pipeline for OAK-D.
Runs YOLOv4-tiny spatial detection + object tracking on-device.
"""

import argparse
import contextlib
import csv
import pathlib
import time

import cv2
import depthai as dai
import yaml

from modules.target_tracking.stereo_node import create_stereo_depth
from modules.target_tracking.spatial_detection_node import create_spatial_detection_network
from modules.target_tracking.object_tracker_node import create_object_tracker

CONFIG_FILE_PATH = pathlib.Path("config.yaml")
OUTPUT_QUEUE_SIZE = 4

# Columns for the accuracy-test CSV (enabled with --log). Ground-truth columns hold the
# ruler-measured target position; raw_* are straight from the device; cal_* are corrected.
LOG_FIELDNAMES = (
    "timestamp",
    "gt_x_mm",
    "gt_y_mm",
    "gt_z_mm",
    "target_id",
    "raw_x_mm",
    "raw_y_mm",
    "raw_z_mm",
    "cal_x_mm",
    "cal_y_mm",
    "cal_z_mm",
)

# Z-bias calibration anchors: (raw_z_mm, offset_mm_to_subtract).
# Measured at 0.5/1.0/1.5/2.0m; final (2200, 0) tapers smoothly to factory calibration.
# Beyond the last anchor the raw camera value is trusted as-is.
# See documentation/accuracy/calibration.png for the fit visualization.
Z_CALIBRATION_ANCHORS = (
    (527.0, 27.5),  # 0.5m
    (1075.0, 75.1),  # 1.0m
    (1573.0, 73.2),  # 1.5m
    (1951.0, -48.7),  # 2.0m
    (2200.0, 0.0),  # taper end — trust factory beyond this
)


def calibrate_z(raw_z: float) -> float:
    """Apply piecewise-linear bias correction to a raw stereo-depth z value (mm)."""
    if raw_z <= Z_CALIBRATION_ANCHORS[0][0]:
        return raw_z - Z_CALIBRATION_ANCHORS[0][1]
    if raw_z >= Z_CALIBRATION_ANCHORS[-1][0]:
        return raw_z
    for (z0, o0), (z1, o1) in zip(Z_CALIBRATION_ANCHORS, Z_CALIBRATION_ANCHORS[1:]):
        if z0 <= raw_z <= z1:
            t = (raw_z - z0) / (z1 - z0)
            return raw_z - (o0 + t * (o1 - o0))
    return raw_z


def calibrate_xy(raw_x: float, raw_y: float, raw_z: float, cal_z: float) -> tuple[float, float]:
    """Scale raw X/Y by the depth-correction ratio.

    The device computes X = raw_z * (u - cx) / fx (and similarly Y), so X and Y are linear in
    the raw depth. Once Z is corrected, X and Y are corrected by the same ratio cal_z / raw_z.
    Returns the raw values unchanged when raw_z is 0 (no valid depth).
    """
    if raw_z == 0:
        return raw_x, raw_y
    ratio = cal_z / raw_z
    return raw_x * ratio, raw_y * ratio


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for optional accuracy logging."""
    parser = argparse.ArgumentParser(description="OAK-D target tracking pipeline.")
    parser.add_argument(
        "--log",
        type=pathlib.Path,
        default=None,
        help="Append per-frame XYZ rows to this CSV (header written if new). Omit to disable.",
    )
    parser.add_argument(
        "--gt-x", type=float, default=0.0, help="Ground-truth target X in mm (default 0)."
    )
    parser.add_argument(
        "--gt-y", type=float, default=0.0, help="Ground-truth target Y in mm (default 0)."
    )
    parser.add_argument(
        "--gt-z", type=float, default=0.0, help="Ground-truth target Z in mm (default 0)."
    )
    return parser.parse_args()


def main() -> int:
    """Run the OAK-D target tracking pipeline."""
    args = parse_args()

    with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    model_name: str = config["spatial_detection"]["model_name"]

    with contextlib.ExitStack() as stack:
        log_writer = None
        if args.log is not None:
            is_new = not args.log.exists() or args.log.stat().st_size == 0
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log_file = stack.enter_context(open(args.log, "a", newline="", encoding="utf-8"))
            log_writer = csv.writer(log_file)
            if is_new:
                log_writer.writerow(LOG_FIELDNAMES)
            print(
                f"Logging XYZ to {args.log} "
                f"(ground truth x={args.gt_x:.0f}mm, y={args.gt_y:.0f}mm, z={args.gt_z:.0f}mm)"
            )

        with dai.Pipeline() as pipeline:
            stereo = create_stereo_depth(pipeline)
            spatial_detection = create_spatial_detection_network(pipeline, stereo, model_name)
            tracker = create_object_tracker(pipeline, spatial_detection)

            tracklet_queue = tracker.out.createOutputQueue(
                maxSize=OUTPUT_QUEUE_SIZE, blocking=False
            )
            preview_queue = tracker.passthroughTrackerFrame.createOutputQueue(
                maxSize=OUTPUT_QUEUE_SIZE, blocking=False
            )

            pipeline.start()
            recording = False
            while pipeline.isRunning():
                tracklets_msg = tracklet_queue.get()
                frame_msg = preview_queue.get()
                frame = frame_msg.getCvFrame()

                tracked = [
                    t
                    for t in tracklets_msg.tracklets
                    if t.status == dai.Tracklet.TrackingStatus.TRACKED
                ]
                # Log only while recording is armed (press 'r') AND exactly one person is in
                # view, so walk-in frames and bystanders can never contaminate a session.
                can_log = log_writer is not None and recording and len(tracked) == 1

                for tracklet in tracked:
                    roi = tracklet.roi.denormalize(frame.shape[1], frame.shape[0])
                    raw_x = tracklet.spatialCoordinates.x
                    raw_y = tracklet.spatialCoordinates.y
                    raw_z = tracklet.spatialCoordinates.z
                    z_mm = calibrate_z(raw_z)
                    cal_x, cal_y = calibrate_xy(raw_x, raw_y, raw_z, z_mm)

                    print(
                        f"Target ID {tracklet.id}: "
                        f"xyz=({raw_x:.0f}mm, {raw_y:.0f}mm, {z_mm:.0f}mm)  "
                        f"bbox=({int(roi.topLeft().x)}, {int(roi.topLeft().y)}, "
                        f"{int(roi.bottomRight().x)}, {int(roi.bottomRight().y)})"
                    )

                    if can_log:
                        log_writer.writerow(
                            (
                                time.time(),
                                args.gt_x,
                                args.gt_y,
                                args.gt_z,
                                tracklet.id,
                                round(raw_x),
                                round(raw_y),
                                round(raw_z),
                                round(cal_x),
                                round(cal_y),
                                round(z_mm),
                            )
                        )

                    cv2.rectangle(
                        frame,
                        (int(roi.topLeft().x), int(roi.topLeft().y)),
                        (int(roi.bottomRight().x), int(roi.bottomRight().y)),
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"ID{tracklet.id} X{raw_x:.0f} Y{raw_y:.0f} Z{z_mm:.0f}",
                        (int(roi.topLeft().x), int(roi.topLeft().y) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )

                if log_writer is not None:
                    if not recording:
                        banner = f"IDLE - press 'r' to record  (target z={args.gt_z:.0f}mm)"
                        banner_color = (0, 215, 255)
                    elif can_log:
                        banner = f"REC  people:1  (target z={args.gt_z:.0f}mm)"
                        banner_color = (0, 255, 0)
                    else:
                        banner = f"REC PAUSED - need exactly 1 person (people:{len(tracked)})"
                        banner_color = (0, 0, 255)
                    cv2.putText(
                        frame, banner, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, banner_color, 2
                    )

                cv2.imshow("Target Tracking", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r") and log_writer is not None:
                    recording = not recording
                    print("RECORDING" if recording else "stopped recording")

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"ERROR: Status code: {result_main}")
    print("Done!")
