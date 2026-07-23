"""
Camera calibration script using a checkerboard pattern.

Usage:
    python camera_calibration.py --squares-x 9 --squares-y 6 --square-size 0.025

    --squares-x    Number of inner corners along the x-axis (default: 9)
    --squares-y    Number of inner corners along the y-axis (default: 6)
    --square-size  Physical size of each square in meters (default: 0.025 = 2.5cm)
    --camera       Camera index (default: 0)
    --min-samples  Minimum number of good frames before calibrating (default: 20)
    --output       Output YAML file path (default: camera_info.yaml)

Headless mode (default):
    Automatically captures frames when a checkerboard is detected and runs
    calibration once enough samples are collected.

GUI mode (--gui):
    SPACE  - Capture current frame (if checkerboard detected)
    c      - Run calibration with collected frames
    q      - Quit
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OpenCV camera calibration with checkerboard")
    parser.add_argument("--squares-x", type=int, default=9, help="Inner corners along x")
    parser.add_argument("--squares-y", type=int, default=6, help="Inner corners along y")
    parser.add_argument("--square-size", type=float, default=0.025, help="Square size in meters")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--min-samples", type=int, default=20, help="Min frames before calibrating")
    parser.add_argument("--output", type=str, default="camera_info.yaml", help="Output YAML file")
    parser.add_argument("--gui", action="store_true", help="Show interactive preview window")
    parser.add_argument(
        "--capture-interval",
        type=float,
        default=1.0,
        help="Seconds between auto-captures in headless mode",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="Max seconds to collect samples in headless mode (0 = no limit)",
    )
    return parser.parse_args()


def build_object_points(squares_x: int, squares_y: int, square_size: float) -> np.ndarray:
    """3D coordinates of checkerboard corners in the checkerboard frame (z=0)."""
    objp = np.zeros((squares_x * squares_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:squares_x, 0:squares_y].T.reshape(-1, 2)
    objp *= square_size
    return objp


def calibrate(
    object_points: list,
    image_points: list,
    image_size: tuple,
) -> tuple[np.ndarray, np.ndarray, list, list]:
    flags = (
        cv2.CALIB_RATIONAL_MODEL
        | cv2.CALIB_FIX_ASPECT_RATIO
    )
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None, flags=flags
    )
    return rms, K, dist, rvecs, tvecs


def save_yaml(path: str, K: np.ndarray, dist: np.ndarray, image_size: tuple) -> None:
    width, height = image_size
    data = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": "camera",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": K.flatten().tolist(),
        },
        "distortion_model": "rational_polynomial",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist.shape[1]),
            "data": dist.flatten().tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).flatten().tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": np.hstack([K, np.zeros((3, 1))]).flatten().tolist(),
        },
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"[calibration] Saved to {path}")


def print_results(rms: float, K: np.ndarray, dist: np.ndarray) -> None:
    print(f"\n{'='*50}")
    print(f"RMS reprojection error: {rms:.4f} px  (< 1.0 is good)")
    print(f"\nCamera matrix K:\n{K}")
    print(f"\nDistortion coefficients:\n{dist}")
    print(f"\nFor camera_node.py CameraInfo:")
    print(f"  K = {K.flatten().tolist()}")
    print(f"  D = {dist.flatten().tolist()}")
    print(f"{'='*50}\n")


def run_calibration(
    all_object_points: list[np.ndarray],
    all_image_points: list[np.ndarray],
    image_size: tuple[int, int],
    output: str,
    min_samples: int,
) -> bool:
    if len(all_object_points) < min_samples:
        print(f"[calibration] Need at least {min_samples} samples (have {len(all_object_points)})")
        return False

    print(f"\n[calibration] Running calibration with {len(all_object_points)} frames...")
    rms, K, dist, _, _ = calibrate(all_object_points, all_image_points, image_size)
    print_results(rms, K, dist)
    save_yaml(output, K, dist, image_size)

    if rms > 1.0:
        print("[warning] RMS error > 1.0 — try recapturing with better coverage")
    else:
        print("[calibration] Done. Use the K and D values in camera_node.py CameraInfo.")
    return True


def capture_frame(
    objp: np.ndarray,
    corners_refined: np.ndarray,
    all_object_points: list[np.ndarray],
    all_image_points: list[np.ndarray],
    image_size: tuple[int, int] | None,
    gray: np.ndarray,
) -> tuple[int, int] | None:
    if image_size is None:
        image_size = (gray.shape[1], gray.shape[0])
    all_object_points.append(objp)
    all_image_points.append(corners_refined)
    print(f"[calibration] Captured frame {len(all_object_points)}")
    return image_size


def run_headless(
    cap: cv2.VideoCapture,
    pattern: tuple[int, int],
    objp: np.ndarray,
    args: argparse.Namespace,
) -> None:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    all_object_points: list[np.ndarray] = []
    all_image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    last_capture_time = 0.0
    start_time = time.monotonic()

    print(f"[calibration] Pattern: {pattern[0]}x{pattern[1]} inner corners")
    print(f"[calibration] Square size: {args.square_size * 100:.1f} cm")
    print(f"[calibration] Need {args.min_samples} samples before calibrating")
    print("[calibration] Headless mode: move checkerboard into view\n")

    while True:
        if args.timeout > 0 and (time.monotonic() - start_time) >= args.timeout:
            print(f"[error] Timed out after {args.timeout:.0f}s with {len(all_object_points)} samples")
            sys.exit(1)

        ret, frame = cap.read()
        if not ret:
            print("[error] Failed to read frame")
            sys.exit(1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)

        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            now = time.monotonic()
            if now - last_capture_time >= args.capture_interval:
                image_size = capture_frame(
                    objp,
                    corners_refined,
                    all_object_points,
                    all_image_points,
                    image_size,
                    gray,
                )
                last_capture_time = now
        elif len(all_object_points) == 0:
            print("[calibration] Waiting for checkerboard...", end="\r")

        if len(all_object_points) >= args.min_samples and image_size is not None:
            if run_calibration(all_object_points, all_image_points, image_size, args.output, args.min_samples):
                return

        time.sleep(0.05)


def run_gui(
    cap: cv2.VideoCapture,
    pattern: tuple[int, int],
    objp: np.ndarray,
    args: argparse.Namespace,
) -> None:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    all_object_points: list[np.ndarray] = []
    all_image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    print(f"[calibration] Pattern: {pattern[0]}x{pattern[1]} inner corners")
    print(f"[calibration] Square size: {args.square_size * 100:.1f} cm")
    print(f"[calibration] Need {args.min_samples} samples before calibrating")
    print("[calibration] Point camera at checkerboard and press SPACE to capture\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[error] Failed to read frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)

        display = frame.copy()
        corners_refined = None

        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, pattern, corners_refined, found)
            status_color = (0, 255, 0)
            status_text = (
                f"Checkerboard found  |  Samples: {len(all_object_points)}/{args.min_samples}  |  SPACE to capture"
            )
        else:
            status_color = (0, 0, 255)
            status_text = f"No checkerboard  |  Samples: {len(all_object_points)}/{args.min_samples}"

        cv2.putText(display, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        if len(all_object_points) >= args.min_samples:
            cv2.putText(
                display,
                "Press 'c' to calibrate",
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 200, 0),
                2,
            )

        cv2.imshow("Camera Calibration", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        if key == ord(" ") and found and corners_refined is not None:
            image_size = capture_frame(
                objp,
                corners_refined,
                all_object_points,
                all_image_points,
                image_size,
                gray,
            )

        if key == ord("c") and image_size is not None:
            if run_calibration(all_object_points, all_image_points, image_size, args.output, args.min_samples):
                break

    cv2.destroyAllWindows()


def main() -> None:
    args = parse_args()

    pattern = (args.squares_x, args.squares_y)
    objp = build_object_points(args.squares_x, args.squares_y, args.square_size)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[error] Cannot open camera {args.camera}")
        sys.exit(1)

    try:
        if args.gui:
            run_gui(cap, pattern, objp, args)
        else:
            run_headless(cap, pattern, objp, args)
    finally:
        cap.release()


if __name__ == "__main__":
    main()
