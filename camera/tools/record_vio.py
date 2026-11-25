"""Record latest OAK-D VIO poses, with a reproducible trial manifest."""

import argparse
import csv
from datetime import datetime, timezone
import importlib.metadata
import json
import logging
import math
from pathlib import Path
import platform
import time

from src.oakd import OakD

CSV_FIELDS = [
    "elapsed_receipt_s",
    "received_monotonic_s",
    "sdk_timestamp_s",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument(
        "--max-silence",
        type=float,
        default=5.0,
        help="Fail if no valid poses arrive for this many recording seconds.",
    )
    parser.add_argument(
        "--camera-model",
        default="not recorded",
        help="Actual model from the camera label; not auto-detected.",
    )
    parser.add_argument("--notes", default="")
    args = parser.parse_args(argv)
    for name in ("duration", "max_silence", "warmup"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != "warmup" and value == 0):
            parser.error(
                f"--{name.replace('_', '-')} must be finite and "
                + ("nonnegative" if name == "warmup" else "positive")
            )
    manifest_path = args.output.with_suffix(".metadata.json")
    if args.output.exists() or manifest_path.exists():
        parser.error("Trial CSV or manifest already exists; choose a new filename")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        sdk_version = importlib.metadata.version("depthai")
    except importlib.metadata.PackageNotFoundError:
        sdk_version = "not installed"
    manifest = {
        "schema_version": 1,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "camera_model_user_supplied": args.camera_model,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "depthai_version": sdk_version,
        "notes": args.notes,
        "duration_requested_s": args.duration,
        "warmup_s": args.warmup,
        "max_silence_s": args.max_silence,
        "csv": args.output.name,
        "position_units": "metres",
        "quaternion_order": "xyzw",
        "coordinate_frame": "SDK local frame; no body/NED/GPS conversion",
        "sampling": "latest-pose polling; intermediate SDK outputs may be discarded",
        "status": "starting",
        "samples": 0,
    }
    camera = OakD(enable_vio=True)
    code = 1
    first_position = last_position = None
    first_receipt = last_receipt = None
    max_distance = max_gap = 0.0
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Reserve both files before touching the device. Never overwrite a trial.
        with args.output.open("x", newline="", encoding="utf-8") as stream:
            with manifest_path.open("x", encoding="utf-8") as metadata:
                writer = csv.writer(stream)
                writer.writerow(CSV_FIELDS)
                try:
                    if not camera.initialize_camera():
                        raise RuntimeError(
                            "Camera initialization failed; inspect SDK logs"
                        )
                    print(f"Keep still during {args.warmup:g}s warmup.", flush=True)
                    warmup_end = time.monotonic() + args.warmup
                    while time.monotonic() < warmup_end:
                        camera.capture_pose()
                        time.sleep(0.005)
                    print(
                        "RECORDING - begin the trial; hold still at both endpoints.",
                        flush=True,
                    )
                    start = last_valid = time.monotonic()
                    end = start + args.duration
                    while time.monotonic() < end:
                        pose = camera.capture_pose()
                        if pose is not None:
                            writer.writerow(
                                [
                                    pose.received_monotonic_s - start,
                                    pose.received_monotonic_s,
                                    pose.timestamp_s,
                                    *pose.position_m,
                                    *pose.orientation_xyzw,
                                ]
                            )
                            stream.flush()
                            manifest["samples"] += 1
                            first_position = first_position or pose.position_m
                            if first_receipt is None:
                                first_receipt = pose.received_monotonic_s
                            if last_receipt is not None:
                                max_gap = max(
                                    max_gap, pose.received_monotonic_s - last_receipt
                                )
                            last_position = pose.position_m
                            last_receipt = pose.received_monotonic_s
                            max_distance = max(
                                max_distance, math.dist(first_position, last_position)
                            )
                            last_valid = time.monotonic()
                        elif time.monotonic() - last_valid >= args.max_silence:
                            raise RuntimeError(
                                "No valid VIO poses within the silence timeout"
                            )
                        time.sleep(0.005)
                    if manifest["samples"] < 2:
                        raise RuntimeError("Insufficient pose samples")
                    manifest["status"] = "complete"
                    code = 0
                except KeyboardInterrupt:
                    manifest["status"] = "interrupted"
                    code = 130
                    print("\nInterrupted; preserving partial trial.")
                except Exception as error:
                    manifest["status"] = "failed"
                    manifest["error"] = str(error)
                    logging.exception("Trial failed")
                finally:
                    camera.close_camera()
                    manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
                    if manifest["samples"] >= 2:
                        manifest["descriptive_metrics"] = {
                            "endpoint_displacement_m": math.dist(
                                first_position, last_position
                            ),
                            "maximum_distance_from_first_pose_m": max_distance,
                            "maximum_receipt_gap_s": max_gap,
                            "receipt_duration_s": last_receipt - first_receipt,
                        }
                    json.dump(manifest, metadata, indent=2, allow_nan=False)
                    metadata.write("\n")
        print(f"Saved {manifest['samples']} poses: {args.output}")
        print(f"Trial status: {manifest['status']}; manifest: {manifest_path}")
        print("Use tools.analyze_vio with measured ground truth to assess accuracy.")
        return code
    except OSError as error:
        logging.error("Could not create trial files: %s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
