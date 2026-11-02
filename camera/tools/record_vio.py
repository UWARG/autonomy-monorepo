"""Record OAK-D VIO poses to CSV for hardware testing."""

import argparse
import csv
import logging
import math
from pathlib import Path
import time

from src.oakd import OakD


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="New CSV file to create.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Recording duration in seconds.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=5.0,
        help="Warmup duration in seconds.",
    )
    args = parser.parse_args()

    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")

    if not math.isfinite(args.warmup) or args.warmup < 0:
        parser.error("--warmup must be finite and nonnegative")

    if args.output.exists():
        parser.error("Output file already exists; choose a new filename")

    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    camera = OakD(enable_vio=True)
    positions = []
    receipt_times = []
    interrupted = False

    try:
        if not camera.initialize_camera():
            logging.error("Camera initialization failed")
            return 1

        args.output.parent.mkdir(parents=True, exist_ok=True)

        # Exclusive creation prevents overwriting earlier trials.
        with args.output.open(
            "x",
            newline="",
            encoding="utf-8",
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
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
            )

            print(
                f"Keep the camera still during "
                f"{args.warmup:g} seconds of warmup.",
                flush=True,
            )

            warmup_end = time.monotonic() + args.warmup

            while time.monotonic() < warmup_end:
                camera.capture_pose()
                time.sleep(0.005)

            print("RECORDING — begin your trial now.", flush=True)
            recording_start = time.monotonic()
            recording_end = recording_start + args.duration

            try:
                while time.monotonic() < recording_end:
                    pose = camera.capture_pose()

                    if pose is not None:
                        writer.writerow(
                            [
                                (
                                    pose.received_monotonic_s
                                    - recording_start
                                ),
                                pose.received_monotonic_s,
                                pose.timestamp_s,
                                *pose.position_m,
                                *pose.orientation_xyzw,
                            ]
                        )
                        positions.append(pose.position_m)
                        receipt_times.append(
                            pose.received_monotonic_s
                        )

                    time.sleep(0.005)

            except KeyboardInterrupt:
                interrupted = True
                print("\nStopped early; preserving recorded samples.")

    finally:
        camera.close_camera()

    print(f"Saved {len(positions)} pose samples to {args.output}")

    if len(positions) < 2:
        logging.error(
            "Insufficient samples to assess VIO. "
            "Inspect the camera and SDK logs."
        )
        return 1

    endpoint_distance = math.dist(positions[0], positions[-1])
    maximum_distance = max(
        math.dist(positions[0], position)
        for position in positions
    )
    maximum_gap = max(
        later - earlier
        for earlier, later in zip(
            receipt_times,
            receipt_times[1:],
        )
    )

    print(
        f"Distance between first and last positions: "
        f"{endpoint_distance:.4f} m"
    )
    print(
        f"Maximum distance from first position: "
        f"{maximum_distance:.4f} m"
    )
    print(f"Maximum host receipt gap: {maximum_gap:.4f} s")
    print(
        "These distances indicate drift only for a stationary "
        "or return-to-start trial."
    )
    print(
        "This records the latest polled poses, not every "
        "estimator output. Receipt gaps are not sensor latency."
    )

    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())