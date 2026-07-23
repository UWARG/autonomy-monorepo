import logging
import time
from typing import Any, Optional, cast

import cv2
import numpy as np

from .abstract_camera import AbstractCamera
from .frame import CameraFrame

logger = logging.getLogger(__name__)

# Prefer auto exposure to avoid washed-out frames caused by invalid manual
# exposure ranges on different camera drivers.
ARDU_DEVICE_INDEX = 0
ARDU_USE_AUTO_EXPOSURE = True
ARDU_MANUAL_EXPOSURE = -6
ARDU_MANUAL_GAIN: Optional[int] = None
ARDU_BRIGHTNESS: Optional[int] = None

# Adaptive software correction when driver-level exposure controls are ignored.
ARDU_ENABLE_SOFTWARE_EXPOSURE_CORRECTION = True
ARDU_TARGET_P95_LUMA = 210.0
ARDU_TARGET_MEDIAN_LUMA = 105.0
ARDU_SOFT_GAIN_MIN = 0.45
ARDU_SOFT_GAIN_MAX = 1.35


class Arducam(AbstractCamera):
    """Arducam camera with outdoor-tuned exposure controls."""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        super().__init__()
        self.width = width
        self.height = height
        self.cap: Optional[cv2.VideoCapture] = None
        self._software_gain = 1.0

    def initialize_camera(self) -> bool:
        try:
            self.cap = self._open_camera()
            self._configure_controls()
            self._warmup()
            return True
        except RuntimeError:
            return False

    def stop(self) -> None:
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
        frame = self._apply_software_exposure_correction(frame)
        return CameraFrame(rgb=frame, depth=None, rgb_down=None)

    def _open_camera(self) -> cv2.VideoCapture:
        for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
            cap = cv2.VideoCapture(ARDU_DEVICE_INDEX, backend)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            fourcc_fn = cast(Any, getattr(cv2, "VideoWriter_fourcc", None))
            if callable(fourcc_fn):
                fourcc_raw = fourcc_fn(*"MJPG")
                if isinstance(fourcc_raw, (int, float)):
                    cap.set(cv2.CAP_PROP_FOURCC, float(fourcc_raw))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            backend_name = "unknown"
            try:
                backend_name = cap.getBackendName()
            except Exception:
                pass
            logger.info(
                "Arducam opened device=%s backend=%s",
                ARDU_DEVICE_INDEX,
                backend_name,
            )
            return cap

        raise RuntimeError(f"Failed to open Arducam at index {ARDU_DEVICE_INDEX}")

    def _set_control(self, prop: int, value: float, label: str) -> None:
        if self.cap is None:
            return
        ok = self.cap.set(prop, value)
        actual = self.cap.get(prop)
        logger.info(
            "Arducam %s request=%s applied=%s actual=%s",
            label,
            value,
            ok,
            actual,
        )

    def _configure_controls(self) -> None:
        auto_values = (0.75, 3.0) if ARDU_USE_AUTO_EXPOSURE else (1.0, 0.25, 0.0)
        for auto_exposure_value in auto_values:
            self._set_control(
                cv2.CAP_PROP_AUTO_EXPOSURE,
                auto_exposure_value,
                "auto_exposure",
            )

        if not ARDU_USE_AUTO_EXPOSURE:
            self._set_control(cv2.CAP_PROP_EXPOSURE, ARDU_MANUAL_EXPOSURE, "exposure")

        if ARDU_MANUAL_GAIN is not None:
            self._set_control(cv2.CAP_PROP_GAIN, ARDU_MANUAL_GAIN, "gain")

        if ARDU_BRIGHTNESS is not None:
            self._set_control(cv2.CAP_PROP_BRIGHTNESS, ARDU_BRIGHTNESS, "brightness")

    def _warmup(self) -> None:
        if self.cap is None:
            return
        for _ in range(25):
            self.cap.read()
            time.sleep(0.02)

    def _apply_software_exposure_correction(self, frame: np.ndarray) -> np.ndarray:
        if not ARDU_ENABLE_SOFTWARE_EXPOSURE_CORRECTION:
            return frame

        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        p95 = float(np.percentile(gray, 95))
        p50 = float(np.percentile(gray, 50))
        highlight_ratio = float(np.mean(gray >= 250))

        target_gain = 1.0
        if p95 > ARDU_TARGET_P95_LUMA or highlight_ratio > 0.05:
            target_gain = ARDU_TARGET_P95_LUMA / max(p95, 1.0)
        elif p50 < ARDU_TARGET_MEDIAN_LUMA:
            target_gain = ARDU_TARGET_MEDIAN_LUMA / max(p50, 1.0)

        target_gain = float(np.clip(target_gain, ARDU_SOFT_GAIN_MIN, ARDU_SOFT_GAIN_MAX))

        self._software_gain = (0.88 * self._software_gain) + (0.12 * target_gain)
        return cv2.convertScaleAbs(frame, alpha=self._software_gain, beta=0.0)

    def _normalize_geometry(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return frame

        scale = min(float(self.width) / float(w), float(self.height) / float(h))
        fit_w = max(1, int(round(w * scale)))
        fit_h = max(1, int(round(h * scale)))
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(frame, (fit_w, fit_h), interpolation=interpolation)

        canvas = np.zeros((self.height, self.width, 3), dtype=frame.dtype)
        x0 = (self.width - fit_w) // 2
        y0 = (self.height - fit_h) // 2
        canvas[y0 : y0 + fit_h, x0 : x0 + fit_w] = resized
        return canvas
