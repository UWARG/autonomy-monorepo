"""Run a repeatable manual smoke test against a physical USB/UVC ArduCam."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from src.arducam import ArduCam
from src.frame import CameraFrame


WARMUP_FRAMES = 10


class SmokeTestError(RuntimeError):
    """Raised when the physical-camera smoke test does not meet its criteria."""


@dataclass(frozen=True)
class SmokeTestResult:
    """Measurements reported after a successful camera run."""

    frames_captured: int
    width: int
    height: int
    elapsed_seconds: float
    frames_per_second: float
    output_path: Path


def positive_int(value: str) -> int:
    """Parse a strictly positive integer for an argparse option."""
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def nonnegative_int(value: str) -> int:
    """Parse a nonnegative integer for an argparse option."""
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be zero or greater")
    return parsed


def validate_frame(
    frame: CameraFrame | None,
    *,
    expected_width: int,
    expected_height: int,
    label: str,
) -> CameraFrame:
    """Validate one frame returned by the ArduCam abstraction."""
    if not isinstance(frame, CameraFrame):
        raise SmokeTestError(f"{label} capture did not return a CameraFrame")

    rgb = frame.rgb
    if not isinstance(rgb, np.ndarray):
        raise SmokeTestError(f"{label} frame RGB data is not a numpy array")
    if rgb.size == 0:
        raise SmokeTestError(f"{label} frame RGB data is empty")
    if rgb.dtype != np.uint8:
        raise SmokeTestError(
            f"{label} frame has dtype {rgb.dtype}; expected uint8"
        )

    expected_shape = (expected_height, expected_width, 3)
    if rgb.shape != expected_shape:
        raise SmokeTestError(
            f"{label} frame has shape {rgb.shape}; expected {expected_shape}"
        )

    return frame


def run_smoke_test(
    *,
    device_index: int,
    width: int,
    height: int,
    frame_count: int,
    output_path: Path,
) -> SmokeTestResult:
    """Capture and validate frames using the production ArduCam class."""
    camera = ArduCam(device_index=device_index, width=width, height=height)
    last_frame: CameraFrame | None = None

    try:
        if not camera.initialize_camera():
            raise SmokeTestError(
                f"could not initialize camera at device index {device_index}"
            )

        for frame_number in range(1, WARMUP_FRAMES + 1):
            validate_frame(
                camera.capture_frame(),
                expected_width=width,
                expected_height=height,
                label=f"warm-up frame {frame_number}/{WARMUP_FRAMES}",
            )

        start_time = time.perf_counter()
        for frame_number in range(1, frame_count + 1):
            last_frame = validate_frame(
                camera.capture_frame(),
                expected_width=width,
                expected_height=height,
                label=f"measured frame {frame_number}/{frame_count}",
            )
        elapsed_seconds = time.perf_counter() - start_time

        if last_frame is None:
            raise SmokeTestError("no measured frame was captured")

        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        bgr = cv2.cvtColor(last_frame.rgb, cv2.COLOR_RGB2BGR)
        try:
            image_saved = cv2.imwrite(str(output_path), bgr)
        except cv2.error as error:
            raise SmokeTestError(f"could not save output image: {error}") from error
        if not image_saved:
            raise SmokeTestError(f"could not save output image to {output_path}")

        frames_per_second = (
            frame_count / elapsed_seconds if elapsed_seconds > 0 else float("inf")
        )
        return SmokeTestResult(
            frames_captured=frame_count,
            width=last_frame.rgb.shape[1],
            height=last_frame.rgb.shape[0],
            elapsed_seconds=elapsed_seconds,
            frames_per_second=frames_per_second,
            output_path=output_path,
        )
    finally:
        camera.stop()


def build_parser() -> argparse.ArgumentParser:
    """Build the smoke-test command-line interface."""
    parser = argparse.ArgumentParser(
        description="Validate the ArduCam abstraction using a physical USB/UVC camera."
    )
    parser.add_argument("--device-index", type=nonnegative_int, default=0)
    parser.add_argument("--width", type=positive_int, default=1280)
    parser.add_argument("--height", type=positive_int, default=720)
    parser.add_argument("--frames", type=positive_int, default=60)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("arducam_smoke.jpg"),
        help="path for the final captured frame",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the CLI and return a process exit code."""
    args = build_parser().parse_args(argv)

    try:
        result = run_smoke_test(
            device_index=args.device_index,
            width=args.width,
            height=args.height,
            frame_count=args.frames,
            output_path=args.output,
        )
    except SmokeTestError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print("PASS: ArduCam hardware smoke test")
    print(f"Device index: {args.device_index}")
    print(f"Frames captured: {result.frames_captured}/{args.frames}")
    print(
        "Requested/actual resolution: "
        f"{args.width}x{args.height} / {result.width}x{result.height}"
    )
    print(f"Elapsed: {result.elapsed_seconds:.3f} seconds")
    print(f"Observed FPS: {result.frames_per_second:.2f}")
    print(f"Saved frame: {result.output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
