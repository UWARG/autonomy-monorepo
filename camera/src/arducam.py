import logging
import shutil
import subprocess
import time
from typing import Any, Optional, cast

import cv2
import numpy as np

from .abstract_camera import AbstractCamera
from .frame import CameraFrame

logger = logging.getLogger(__name__)

ARDU_DEVICE_INDEX = 0

# Prefer firmware AE outdoors — on OV9782, exposure_time_absolute=1 still
# clips in sun; AE often drives the sensor harder than the UVC absolute knob.
ARDU_PREFER_AUTO_EXPOSURE = True
# With AE, don't also pin brightness/gamma at the floor (causes purple cast /
# underexposure). Keep gain at 0 so AE can't crank ISO.
ARDU_MANUAL_EXPOSURE = 1
ARDU_MANUAL_GAIN = 0
ARDU_BRIGHTNESS = -20
ARDU_GAMMA = 100
ARDU_CONTRAST = 32

# Software tonemap when the sensor still clips (common in bright sun).
ARDU_ENABLE_SOFTWARE_EXPOSURE_CORRECTION = True
ARDU_TARGET_P95_LUMA = 190.0
ARDU_TARGET_MEDIAN_LUMA = 95.0
ARDU_SOFT_GAIN_MIN = 0.05
ARDU_SOFT_GAIN_MAX = 1.15
ARDU_HIGHLIGHT_CLIP_RATIO = 0.08


class Arducam(AbstractCamera):
    """Arducam OV9782 with outdoor-safe exposure (V4L2 + software tonemap)."""

    def __init__(self, width: int = 1280, height: int = 720) -> None:
        super().__init__()
        self.width = width
        self.height = height
        self.cap: Optional[cv2.VideoCapture] = None
        self._software_gain = 1.0
        self._device = f"/dev/video{ARDU_DEVICE_INDEX}"

    def initialize_camera(self) -> bool:
        try:
            # Pre-open format/controls (may be overwritten when VideoCapture starts).
            self._v4l2_set_format()
            self._apply_v4l2_outdoor_controls(prefer_auto=ARDU_PREFER_AUTO_EXPOSURE)
            self.cap = self._open_camera()
            # Grab a few frames so the UVC stream is actually running…
            self._drain_frames(5)
            # …then lock controls again (this is the step that usually sticks).
            self._apply_v4l2_outdoor_controls(prefer_auto=ARDU_PREFER_AUTO_EXPOSURE)
            self._configure_opencv_controls()
            self._drain_frames(20)
            self._apply_v4l2_outdoor_controls(prefer_auto=ARDU_PREFER_AUTO_EXPOSURE)
            self._log_v4l2_state()
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

    def _run_v4l2(self, args: list[str]) -> bool:
        if shutil.which("v4l2-ctl") is None:
            return False
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", self._device, *args],
                check=True,
                capture_output=True,
                text=True,
            )
            return True
        except (OSError, subprocess.CalledProcessError) as exc:
            logger.warning("v4l2-ctl %s failed: %s", " ".join(args), exc)
            return False

    def _v4l2_set_format(self) -> None:
        self._run_v4l2(
            [
                f"--set-fmt-video=width={self.width},height={self.height},pixelformat=MJPG",
            ]
        )

    def _apply_v4l2_outdoor_controls(self, *, prefer_auto: bool) -> None:
        """Apply outdoor controls. Must run while streaming for OV9782/UVC."""
        if shutil.which("v4l2-ctl") is None:
            logger.warning("v4l2-ctl not found; skipping native outdoor control setup")
            return

        # Shared "dark" knobs either way.
        self._run_v4l2([f"--set-ctrl=brightness={ARDU_BRIGHTNESS}"])
        self._run_v4l2([f"--set-ctrl=gain={ARDU_MANUAL_GAIN}"])
        self._run_v4l2([f"--set-ctrl=gamma={ARDU_GAMMA}"])
        self._run_v4l2([f"--set-ctrl=contrast={ARDU_CONTRAST}"])
        self._run_v4l2(["--set-ctrl=backlight_compensation=0"])
        self._run_v4l2(["--set-ctrl=exposure_dynamic_framerate=0"])

        if prefer_auto:
            # 3 = Aperture Priority (firmware AE). Often the only way this
            # camera stays usable outdoors; absolute=1 still blows out.
            self._run_v4l2(["--set-ctrl=auto_exposure=3"])
            mode = "auto(3)"
        else:
            self._run_v4l2(["--set-ctrl=auto_exposure=1"])
            self._run_v4l2(
                [f"--set-ctrl=exposure_time_absolute={ARDU_MANUAL_EXPOSURE}"]
            )
            mode = f"manual exposure={ARDU_MANUAL_EXPOSURE}"

        logger.info(
            "V4L2 outdoor controls applied (%s brightness=%s gain=%s gamma=%s)",
            mode,
            ARDU_BRIGHTNESS,
            ARDU_MANUAL_GAIN,
            ARDU_GAMMA,
        )

    def _log_v4l2_state(self) -> None:
        if shutil.which("v4l2-ctl") is None:
            return
        try:
            out = subprocess.run(
                [
                    "v4l2-ctl",
                    "-d",
                    self._device,
                    "--get-ctrl=auto_exposure,exposure_time_absolute,brightness,gain,gamma",
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            logger.info("V4L2 state:\n%s", out.stdout.strip())
        except (OSError, subprocess.CalledProcessError):
            pass

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
            # Higher FPS can force shorter integration on some UVC bridges.
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

    def _configure_opencv_controls(self) -> None:
        """Light OpenCV reinforcement — prefer V4L2 as source of truth."""
        if self.cap is None:
            return

        if ARDU_PREFER_AUTO_EXPOSURE:
            # V4L2 aperture-priority ≈ OpenCV auto 0.75 / 3.0 depending on backend.
            for value in (0.75, 3.0):
                self._set_control(cv2.CAP_PROP_AUTO_EXPOSURE, value, "auto_exposure")
        else:
            for value in (1.0, 0.25):
                self._set_control(cv2.CAP_PROP_AUTO_EXPOSURE, value, "auto_exposure")
            self._set_control(cv2.CAP_PROP_EXPOSURE, float(ARDU_MANUAL_EXPOSURE), "exposure")

        self._set_control(cv2.CAP_PROP_GAIN, float(ARDU_MANUAL_GAIN), "gain")
        self._set_control(cv2.CAP_PROP_BRIGHTNESS, float(ARDU_BRIGHTNESS), "brightness")
        self._set_control(cv2.CAP_PROP_GAMMA, float(ARDU_GAMMA), "gamma")
        self._set_control(cv2.CAP_PROP_CONTRAST, float(ARDU_CONTRAST), "contrast")

    def _drain_frames(self, count: int) -> None:
        if self.cap is None:
            return
        for _ in range(count):
            self.cap.read()
            time.sleep(0.01)

    def _apply_software_exposure_correction(self, frame: np.ndarray) -> np.ndarray:
        if not ARDU_ENABLE_SOFTWARE_EXPOSURE_CORRECTION:
            return frame

        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        p99 = float(np.percentile(gray, 99))
        p95 = float(np.percentile(gray, 95))
        p50 = float(np.percentile(gray, 50))
        highlight_ratio = float(np.mean(gray >= 250))

        target_gain = 1.0
        if highlight_ratio >= ARDU_HIGHLIGHT_CLIP_RATIO:
            # Frame is largely clipped — crush hard so tags/ground regain contrast.
            # Use non-clipped pixels when available; otherwise force min gain.
            unclipped = gray[gray < 250]
            if unclipped.size > 50:
                ref = float(np.percentile(unclipped, 95))
                target_gain = ARDU_TARGET_P95_LUMA / max(ref, 1.0)
            else:
                target_gain = ARDU_SOFT_GAIN_MIN
            # Extra crush proportional to how much of the frame is blown out.
            target_gain *= float(np.clip(1.0 - 0.7 * highlight_ratio, 0.15, 1.0))
        elif p95 > ARDU_TARGET_P95_LUMA or p99 > 245:
            target_gain = ARDU_TARGET_P95_LUMA / max(p95, 1.0)
        elif p50 < ARDU_TARGET_MEDIAN_LUMA:
            target_gain = ARDU_TARGET_MEDIAN_LUMA / max(p50, 1.0)

        target_gain = float(np.clip(target_gain, ARDU_SOFT_GAIN_MIN, ARDU_SOFT_GAIN_MAX))
        # Faster adaptation outdoors so the first published frames aren't white.
        self._software_gain = (0.70 * self._software_gain) + (0.30 * target_gain)

        corrected = cv2.convertScaleAbs(frame, alpha=self._software_gain, beta=0.0)

        # Mild local contrast so AprilTags pop after global darkening.
        if self._software_gain < 0.85:
            lab = cv2.cvtColor(corrected, cv2.COLOR_BGR2LAB)
            luminance, a_ch, b_ch = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            luminance = clahe.apply(luminance)
            corrected = cv2.cvtColor(cv2.merge([luminance, a_ch, b_ch]), cv2.COLOR_LAB2BGR)

        return corrected

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
