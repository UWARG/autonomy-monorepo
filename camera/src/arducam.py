import logging
import time
from typing import Optional

import cv2
import numpy as np

from .abstract_camera import AbstractCamera
from .constants import ARDU_DEVICE_INDEX, ARDUCAM_DEFAULT_HEIGHT, ARDUCAM_DEFAULT_WIDTH
from .frame import CameraFrame

logger = logging.getLogger(__name__)


class Arducam(AbstractCamera):
    """Module for Arducam"""

    def __init__(
        self,
        width: int = ARDUCAM_DEFAULT_WIDTH,
        height: int = ARDUCAM_DEFAULT_HEIGHT,
    ) -> None:
        super().__init__(width=width, height=height)
        self.cap: Optional[cv2.VideoCapture] = None

    def initialize_camera(self) -> bool:
        try:
            self.cap = self._open_camera()

            # Check UVC is running by taking a few photos
            self._drain_frames(5)
            return True
        except RuntimeError:
            return False

    def close_camera(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def capture_frame(self) -> CameraFrame | None:
        if self.cap is None:
            return None

        ret, frame = self.cap.read()
        if not ret:
            return None

        frame = self._normalize_geometry(frame)
        return CameraFrame(rgb=frame)

    def _open_camera(self) -> cv2.VideoCapture:
        for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
            cap = cv2.VideoCapture(ARDU_DEVICE_INDEX, backend)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FPS, 60)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                backend_name = "unknown"
                try:
                    backend_name = cap.getBackendName()
                except Exception:
                    pass
                logger.info(
                    "Arducam opened device=%s backend=%s size=%sx%s",
                    ARDU_DEVICE_INDEX,
                    backend_name,
                    int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                )
                return cap

            cap.release()

        raise RuntimeError(f"Failed to open Arducam at index {ARDU_DEVICE_INDEX}")

    def _drain_frames(self, count: int) -> None:
        if self.cap is None:
            return
        for _ in range(count):
            self.cap.read()
            time.sleep(0.01)

    def _normalize_geometry(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return frame

        scale = min(float(self._width) / float(w), float(self._height) / float(h))
        fit_w = max(1, int(round(w * scale)))
        fit_h = max(1, int(round(h * scale)))
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(frame, (fit_w, fit_h), interpolation=interpolation)

        canvas = np.zeros((self._height, self._width, 3), dtype=frame.dtype)
        x0 = (self._width - fit_w) // 2
        y0 = (self._height - fit_h) // 2
        canvas[y0 : y0 + fit_h, x0 : x0 + fit_w] = resized
        return canvas
