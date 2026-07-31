"""Quick capture smoke test for Arducam.

Run from anywhere:
  python3 camera/src/test_camera.py
or from camera/src:
  python3 test_camera.py
"""

from __future__ import annotations

import sys
from importlib import import_module
from pathlib import Path

import cv2

# Load as package `src.*` so arducam's relative imports work when run as a script.
_CAMERA_DIR = Path(__file__).resolve().parent.parent
if str(_CAMERA_DIR) not in sys.path:
    sys.path.insert(0, str(_CAMERA_DIR))

Arducam = import_module("src.arducam").Arducam


def main() -> None:
    camera = Arducam()
    if not camera.initialize_camera():
        raise SystemExit("Failed to open Arducam")

    frame = camera.capture_frame()
    if frame is None:
        camera.stop()
        raise SystemExit("Failed to capture frame")

    # OpenCV frames are already BGR; CameraFrame.rgb stores that buffer.
    out = Path(__file__).resolve().parent / "test_camera.jpg"
    cv2.imwrite(str(out), frame.rgb)
    camera.stop()
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
