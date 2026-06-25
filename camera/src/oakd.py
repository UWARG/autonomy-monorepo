from __future__ import annotations

import logging
import math
from datetime import timedelta
from typing import Any

import numpy as np

from .abstract_camera import AbstractCamera
from .frame import CameraFrame

logger = logging.getLogger(__name__)

try:
    import depthai as dai
except ImportError:
    dai = None  # type: ignore[assignment,misc]


def _require_depthai() -> Any:
    if dai is None:
        raise ImportError(
            "depthai is required for OakDCamera. "
            "Install with: apt install ros-humble-depthai (DepthAI v2)."
        )
    return dai


def _assert_depthai_v2() -> None:
    module = _require_depthai()
    major = int(module.__version__.split(".")[0])
    if major >= 3:
        raise RuntimeError(
            f"depthai {module.__version__} is v3; OakDCamera requires v2. "
            "Use ros-humble-depthai and remove pip install depthai."
        )


def _available_devices() -> list[Any]:
    module = _require_depthai()
    return module.Device.getAllAvailableDevices()


def _apply_stereo_preset(stereo: Any) -> None:
    module = _require_depthai()
    preset_mode = module.node.StereoDepth.PresetMode
    if hasattr(preset_mode, "HIGH_DENSITY"):
        stereo.setDefaultProfilePreset(preset_mode.HIGH_DENSITY)
    elif hasattr(preset_mode, "HIGH_ACCURACY"):
        stereo.setDefaultProfilePreset(preset_mode.HIGH_ACCURACY)


class OakDCamera(AbstractCamera):
    """OAK-D camera using the DepthAI v2 API (ColorCamera, MonoCamera, XLinkOut)."""

    def __init__(
        self,
        rgb_resolution: Any | None = None,
        fps: float = 30.0,
        preview_width: int = 640,
        preview_height: int = 480,
        *,
        force_usb2: bool = False,
    ) -> None:
        super().__init__()
        _assert_depthai_v2()
        module = _require_depthai()
        self._rgb_resolution = (
            rgb_resolution or module.ColorCameraProperties.SensorResolution.THE_1080_P
        )
        self._fps = fps
        self._preview_width = preview_width
        self._preview_height = preview_height
        self._force_usb2 = force_usb2
        self._device: Any | None = None
        self._rgb_queue: Any | None = None
        self._depth_queue: Any | None = None

    def initialize_camera(self) -> bool:
        devices = _available_devices()
        if not devices:
            logger.error(
                "No OAK devices found. Check USB cable/port, host udev rules "
                "(03e7/2184), and Docker device passthrough."
            )
            return False

        logger.info("Found %d OAK device(s)", len(devices))

        if self._force_usb2:
            return self._try_open(usb2=True)

        if self._try_open(usb2=False):
            return True

        logger.warning("USB3 open failed; retrying with USB2")
        self.stop()
        return self._try_open(usb2=True)

    def _try_open(self, usb2: bool) -> bool:
        try:
            pipeline = self._build_pipeline()
            self._device = self._open_device(pipeline, usb2=usb2)
            self._rgb_queue = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self._depth_queue = self._device.getOutputQueue("depth", maxSize=4, blocking=False)
        except Exception:
            logger.exception("Failed to initialize OAK camera (usb2=%s)", usb2)
            self.stop()
            return False

        logger.info("OAK camera initialized (usb2=%s)", usb2)
        return True

    def _open_device(self, pipeline: Any, *, usb2: bool) -> Any:
        module = _require_depthai()
        device_info = module.Device.getFirstAvailableDevice()
        if device_info is None:
            raise RuntimeError("No OAK device available")

        if not usb2:
            return module.Device(pipeline, device_info)

        if hasattr(module, "UsbSpeed"):
            return module.Device(pipeline, device_info, maxUsbSpeed=module.UsbSpeed.HIGH)
        return module.Device(pipeline, device_info, usb2Mode=True)

    def capture_frame(self) -> CameraFrame | None:
        if self._rgb_queue is None:
            return None

        timeout = timedelta(seconds=max(1.0 / self._fps, 0.05))
        rgb_packet = self._get_rgb_packet(timeout)
        if rgb_packet is None:
            return None

        rgb = rgb_packet.getCvFrame()
        rgb = rgb[:, :, ::-1].copy()  # BGR -> RGB

        depth = None
        centre_depth = math.nan
        if self._depth_queue is not None:
            depth_packet = self._depth_queue.tryGet()
            if depth_packet is not None:
                depth = depth_packet.getFrame()
                h, w = depth.shape
                centre_depth = float(depth[h // 2, w // 2]) / 1000.0

        return CameraFrame(
            rgb=rgb,
            depth=depth,
            rgb_down=None,
            centre_depth=centre_depth,
        )

    def _get_rgb_packet(self, timeout: timedelta) -> Any | None:
        try:
            return self._rgb_queue.tryGet(block=True, timeout=timeout)
        except TypeError:
            # Older depthai v2 without block/timeout on tryGet.
            return self._rgb_queue.get()

    def stop(self) -> None:
        self._rgb_queue = None
        self._depth_queue = None
        if self._device is not None:
            self._device.close()
            self._device = None

    def _build_pipeline(self) -> Any:
        module = _require_depthai()
        pipeline = module.Pipeline()

        cam_rgb = pipeline.create(module.node.ColorCamera)
        cam_rgb.setBoardSocket(module.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(self._rgb_resolution)
        cam_rgb.setFps(self._fps)
        cam_rgb.setPreviewSize(self._preview_width, self._preview_height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(module.ColorCameraProperties.ColorOrder.BGR)

        mono_left = pipeline.create(module.node.MonoCamera)
        mono_right = pipeline.create(module.node.MonoCamera)
        mono_left.setBoardSocket(module.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(module.CameraBoardSocket.CAM_C)
        mono_left.setResolution(module.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(module.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(self._fps)
        mono_right.setFps(self._fps)

        stereo = pipeline.create(module.node.StereoDepth)
        _apply_stereo_preset(stereo)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.create(module.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_depth = pipeline.create(module.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline
