"""Check SDK support; optionally require real RGB, depth, and VIO output."""

import argparse
import logging
import math
import time

from src.oakd import OakD


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Open the camera and check RGB/depth plus VIO output",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    logging.basicConfig(level=logging.INFO)
    try:
        import depthai as dai
    except ImportError:
        print("DepthAI is missing. Run: uv sync --locked --extra dev --extra oakd")
        return 1
    available = hasattr(dai.node, "BasaltVIO")
    print(f"DepthAI {dai.__version__}; BasaltVIO available: {available}")
    if not available:
        print("Use a Basalt-enabled SDK build. The tested Windows wheel omitted it.")
        return 1
    if not args.hardware:
        print(
            "SDK support check passed. Camera operation and accuracy remain untested."
        )
        return 0
    camera = OakD(enable_vio=True)
    poses = frames = 0
    try:
        if not camera.initialize_camera():
            return 1
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if camera.capture_pose() is not None:
                poses += 1
            if camera.capture_frame() is not None:
                frames += 1
            if poses >= 2 and frames >= 2:
                print(
                    "Received RGB/depth frames and valid poses. Accuracy still needs trials."
                )
                return 0
            time.sleep(0.005)
        print(f"Hardware output check failed: frames={frames}, poses={poses}")
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        camera.close_camera()


if __name__ == "__main__":
    raise SystemExit(main())
