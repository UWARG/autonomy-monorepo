"""Capture paired raw/calibrated XYZ from the exact production OAK-D pipeline."""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from contextlib import ExitStack
from pathlib import Path

_CAMERA_ROOT = str(Path(__file__).resolve().parents[1])
if _CAMERA_ROOT not in sys.path:
    sys.path.insert(0, _CAMERA_ROOT)

from src.oakd_follow_pipeline import build_follow_pipeline  # noqa: E402
from src.target_source import (  # noqa: E402
    DepthAITrackletProvider,
    calibrate_xy,
    calibrate_z,
    select_closest_tracked,
)

FIELDS = (
    "session",
    "pose_id",
    "gt_x_mm",
    "gt_y_mm",
    "gt_z_mm",
    "raw_x_mm",
    "raw_y_mm",
    "raw_z_mm",
    "cal_x_mm",
    "cal_y_mm",
    "cal_z_mm",
    "track_id",
    "tracker_sequence_num",
    "detector_sequence_num",
    "tracker_capture_time_s",
    "detector_capture_time_s",
    "host_receipt_time_s",
    "detector_confirmed",
    "camera_fps",
    "detector_stride",
)


def _pose_id(x_mm: float, y_mm: float, z_mm: float) -> str:
    return f"x{x_mm:+g}_y{y_mm:+g}_z{z_mm:g}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Record one calibration pose. Ground truth must be measured from the "
            "camera optical center to the documented target reference point."
        )
    )
    parser.add_argument("--blob-path", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--session", required=True, choices=("fit", "validation"))
    parser.add_argument("--gt-x-mm", required=True, type=float)
    parser.add_argument("--gt-y-mm", required=True, type=float)
    parser.add_argument("--gt-z-mm", required=True, type=float)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--person-label", type=int, default=15)
    parser.add_argument("--camera-fps", type=int, default=20)
    parser.add_argument("--detector-stride", type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    if args.samples < 100:
        raise SystemExit("--samples must be at least 100")
    if (
        not all(
            math.isfinite(value) for value in (args.gt_x_mm, args.gt_y_mm, args.gt_z_mm)
        )
        or args.gt_z_mm <= 0.0
    ):
        raise SystemExit("ground-truth XYZ must be finite and Z must be positive")

    import depthai as dai

    parent = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(parent, exist_ok=True)
    exists = os.path.exists(args.out) and os.path.getsize(args.out) > 0
    pipeline = build_follow_pipeline(
        args.blob_path,
        person_label=args.person_label,
        camera_fps=args.camera_fps,
        detector_stride=args.detector_stride,
    )
    pose_id = _pose_id(args.gt_x_mm, args.gt_y_mm, args.gt_z_mm)
    locked_id = None
    captured = 0

    with ExitStack() as stack:
        device = stack.enter_context(dai.Device(pipeline))
        output = stack.enter_context(open(args.out, "a", newline="", encoding="utf-8"))
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        if not exists:
            writer.writeheader()
        provider = DepthAITrackletProvider(
            device.getOutputQueue("tracklets", maxSize=8, blocking=False),
            detector_queue=device.getOutputQueue(
                "detector_frames", maxSize=8, blocking=False
            ),
            host_sync_now_fn=dai.Clock.now,
        )
        print(
            f"Capturing {args.session}/{pose_id}: {args.samples} detector-confirmed "
            "samples. Keep the rig and the same physical target reference point fixed."
        )
        while captured < args.samples:
            packet = provider.poll()
            if packet is None:
                time.sleep(0.001)
                continue
            if not packet.detector_confirmed:
                continue
            if locked_id is None:
                selected = select_closest_tracked(
                    packet.tracklets, "TRACKED", max_range_mm=None
                )
                if selected is None:
                    continue
                locked_id = int(selected.id)
                print(f"Locked track ID {locked_id}")
            tracklet = next(
                (
                    item
                    for item in packet.tracklets
                    if int(item.id) == locked_id
                    and str(getattr(item.status, "name", item.status)).upper()
                    == "TRACKED"
                ),
                None,
            )
            if tracklet is None:
                continue
            spatial = tracklet.spatialCoordinates
            raw_x = float(spatial.x)
            raw_y = float(spatial.y)
            raw_z = float(spatial.z)
            if not all(math.isfinite(value) for value in (raw_x, raw_y, raw_z)):
                continue
            cal_z = calibrate_z(raw_z)
            cal_x, cal_y = calibrate_xy(raw_x, raw_y, raw_z, cal_z)
            writer.writerow(
                {
                    "session": args.session,
                    "pose_id": pose_id,
                    "gt_x_mm": args.gt_x_mm,
                    "gt_y_mm": args.gt_y_mm,
                    "gt_z_mm": args.gt_z_mm,
                    "raw_x_mm": raw_x,
                    "raw_y_mm": raw_y,
                    "raw_z_mm": raw_z,
                    "cal_x_mm": cal_x,
                    "cal_y_mm": cal_y,
                    "cal_z_mm": cal_z,
                    "track_id": locked_id,
                    "tracker_sequence_num": packet.sequence_num,
                    "detector_sequence_num": packet.detector_sequence_num,
                    "tracker_capture_time_s": f"{packet.capture_time_s:.9f}",
                    "detector_capture_time_s": (
                        f"{packet.detector_capture_time_s:.9f}"
                        if packet.detector_capture_time_s is not None
                        else ""
                    ),
                    "host_receipt_time_s": f"{packet.received_time_s:.9f}",
                    "detector_confirmed": 1,
                    "camera_fps": args.camera_fps,
                    "detector_stride": args.detector_stride,
                }
            )
            captured += 1
            if captured % 25 == 0:
                output.flush()
                print(f"  {captured}/{args.samples}")
    print(f"Wrote {captured} samples to {args.out}")


if __name__ == "__main__":
    main()
