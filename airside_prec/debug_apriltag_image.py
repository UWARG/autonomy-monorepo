#!/usr/bin/env python3
"""Detect AprilTags like the airside_prec ROS stack, draw boxes, print x,y,z.

By default this mirrors the container pipeline:
  - letterbox/resize to camera_node 640x480 (Arducam output)
  - camera_info.yaml K/D as published (NOT scaled) — same as camera_node.py
  - tag size / family / detector params from apriltag.yaml

Usage:
  uv run python debug_apriltag_image.py images/output.jpg
  uv run python debug_apriltag_image.py ../camera/src/test_camera.jpg --correct-intrinsics

Pose is camera optical frame (X right, Y down, Z out of lens), meters.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from pupil_apriltags import Detector
except ImportError:
    print(
        "Missing pupil_apriltags. From airside_prec/:\n"
        "  uv run python debug_apriltag_image.py <image>",
        file=sys.stderr,
    )
    raise SystemExit(1)

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CAMERA_INFO = SCRIPT_DIR / "src" / "camera_info.yaml"
DEFAULT_APRILTAG_YAML = SCRIPT_DIR / "src" / "apriltag.yaml"

# Match airside_prec camera_node + Arducam defaults.
ROS_IMAGE_WIDTH = 640
ROS_IMAGE_HEIGHT = 480


def load_apriltag_config(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    # File shape: apriltag: { ros__parameters: { ... } }
    params = raw.get("apriltag", raw)
    if "ros__parameters" in params:
        params = params["ros__parameters"]
    detector = params.get("detector", {}) or {}
    family = str(params.get("family", "36h11"))
    if not family.startswith("tag"):
        family = f"tag{family}"
    return {
        "family": family,
        "size": float(params.get("size", 0.165)),
        "threads": int(detector.get("threads", 1)),
        "decimate": float(detector.get("decimate", 2.0)),
        "blur": float(detector.get("blur", 0.0)),
        "refine": bool(detector.get("refine", True)),
        "sharpening": float(detector.get("sharpening", 0.25)),
    }


def load_camera_info(path: Path) -> tuple[np.ndarray, np.ndarray, int, int]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(1, -1)
    return k, d, int(data["width"]), int(data["height"])


def letterbox(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Same geometry path as camera.src.arducam.Arducam._normalize_geometry."""
    h, w = frame.shape[:2]
    if h <= 0 or w <= 0:
        return frame
    scale = min(float(width) / float(w), float(height) / float(h))
    fit_w = max(1, int(round(w * scale)))
    fit_h = max(1, int(round(h * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (fit_w, fit_h), interpolation=interpolation)
    canvas = np.zeros((height, width, 3), dtype=frame.dtype)
    x0 = (width - fit_w) // 2
    y0 = (height - fit_h) // 2
    canvas[y0 : y0 + fit_h, x0 : x0 + fit_w] = resized
    return canvas


def scale_intrinsics(
    k: np.ndarray, calib_w: int, calib_h: int, img_w: int, img_h: int
) -> np.ndarray:
    sx = img_w / float(calib_w)
    sy = img_h / float(calib_h)
    out = k.copy()
    out[0, 0] *= sx
    out[0, 2] *= sx
    out[1, 1] *= sy
    out[1, 2] *= sy
    return out


def draw_tag(frame: np.ndarray, corners: np.ndarray, tag_id: int, tvec: np.ndarray) -> None:
    pts = corners.astype(int)
    for i in range(4):
        cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 0), 2)
    center = pts.mean(axis=0).astype(int)
    cv2.circle(frame, tuple(center), 4, (0, 0, 255), -1)
    label = f"id={tag_id} x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f}"
    cv2.putText(
        frame,
        label,
        (center[0] + 8, max(12, center[1] - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Input image path")
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--camera-info", type=Path, default=DEFAULT_CAMERA_INFO)
    parser.add_argument("--apriltag-yaml", type=Path, default=DEFAULT_APRILTAG_YAML)
    parser.add_argument(
        "--width",
        type=int,
        default=ROS_IMAGE_WIDTH,
        help=f"ROS publish width (default {ROS_IMAGE_WIDTH})",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=ROS_IMAGE_HEIGHT,
        help=f"ROS publish height (default {ROS_IMAGE_HEIGHT})",
    )
    parser.add_argument(
        "--no-resize",
        action="store_true",
        help="Keep native image size (do not letterbox to ROS 640x480)",
    )
    parser.add_argument(
        "--correct-intrinsics",
        action="store_true",
        help="Scale K to the working image size (NOT what camera_node publishes)",
    )
    parser.add_argument(
        "--size",
        type=float,
        default=None,
        help="Override tag size in meters (default: apriltag.yaml)",
    )
    args = parser.parse_args()

    if not args.image.is_file():
        raise SystemExit(f"Image not found: {args.image}")

    cfg = load_apriltag_config(args.apriltag_yaml)
    tag_size = float(args.size) if args.size is not None else cfg["size"]

    frame = cv2.imread(str(args.image))
    if frame is None:
        raise SystemExit(f"Failed to read image: {args.image}")

    native_h, native_w = frame.shape[:2]
    if args.no_resize:
        work = frame
        mode = "native resolution"
    else:
        work = letterbox(frame, args.width, args.height)
        mode = f"letterboxed to {args.width}x{args.height} (ROS camera_node)"

    img_h, img_w = work.shape[:2]
    k_raw, dist_coeffs, calib_w, calib_h = load_camera_info(args.camera_info)

    if args.correct_intrinsics:
        camera_matrix = scale_intrinsics(k_raw, calib_w, calib_h, img_w, img_h)
        k_mode = f"K scaled {calib_w}x{calib_h} -> {img_w}x{img_h}"
    else:
        camera_matrix = k_raw
        k_mode = (
            f"K as published by camera_node (calib {calib_w}x{calib_h}, "
            f"image {img_w}x{img_h}, NO scale)"
        )

    print(f"Input: {args.image} ({native_w}x{native_h})")
    print(f"Work image: {img_w}x{img_h} — {mode}")
    print(f"Intrinsics: {k_mode}")
    print(
        f"Detector: family={cfg['family']} size={tag_size} m "
        f"decimate={cfg['decimate']} refine={cfg['refine']} "
        f"sharpening={cfg['sharpening']} (from {args.apriltag_yaml.name})"
    )
    print(f"fx={camera_matrix[0,0]:.2f} fy={camera_matrix[1,1]:.2f} "
          f"cx={camera_matrix[0,2]:.2f} cy={camera_matrix[1,2]:.2f}")

    object_points = np.array(
        [
            [-tag_size / 2, tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [-tag_size / 2, -tag_size / 2, 0],
        ],
        dtype=np.float32,
    )

    detector = Detector(
        families=cfg["family"],
        nthreads=cfg["threads"],
        quad_decimate=cfg["decimate"],
        quad_sigma=cfg["blur"],
        refine_edges=1 if cfg["refine"] else 0,
        decode_sharpening=cfg["sharpening"],
        debug=0,
    )

    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    out = args.output or args.image.with_name(args.image.stem + "_apriltag.jpg")

    if not results:
        cv2.imwrite(str(out), work)
        print("No AprilTags detected.")
        print(f"Wrote (unannotated) {out}")
        raise SystemExit(2)

    annotated = work.copy()
    fx = float(camera_matrix[0, 0])

    for det in results:
        corners = np.asarray(det.corners, dtype=np.float32)
        sides = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
        side_px = float(np.mean(sides))
        z_est = fx * tag_size / max(side_px, 1e-6)

        success, _rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            print(f"tag id={det.tag_id}: solvePnP failed")
            continue

        t = tvec.reshape(3)
        print(
            f"tag id={det.tag_id}  "
            f"x={t[0]:+.4f} m  y={t[1]:+.4f} m  z={t[2]:+.4f} m  "
            f"(Z_est≈{z_est:.4f} m)  "
            f"centre_px=({det.center[0]:.1f},{det.center[1]:.1f})"
        )
        print("  frame: camera optical (X right, Y down, Z out of lens)")
        draw_tag(annotated, corners, det.tag_id, t)

    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), annotated):
        raise SystemExit(f"Failed to write {out}")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
