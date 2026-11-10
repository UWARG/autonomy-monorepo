from __future__ import annotations

import json
import math
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.srv import CapturePhoto
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

from camera.src.abstract_camera import AbstractCamera
from camera.src.sim import SimCamera


def _coordinate_record(msg: CoordinateMsg) -> dict[str, float]:
    return {"lat": msg.lat, "lon": msg.lon, "alt": msg.alt}


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _attitude_record(imu: Imu, frame_stamp_ns: int) -> dict:
    stamp_ns = _stamp_ns(imu.header.stamp)
    return {
        "stamp_ns": stamp_ns,
        "frame_offset_ns": stamp_ns - frame_stamp_ns if stamp_ns else None,
        "frame_id": imu.header.frame_id,
        "orientation": {axis: getattr(imu.orientation, axis) for axis in "xyzw"},
        "angular_velocity": {axis: getattr(imu.angular_velocity, axis) for axis in "xyz"},
    }


def _rectified_rgb(msg: Image) -> np.ndarray:
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported rectified image encoding: {msg.encoding}")
    row_width = msg.width * channels
    if not msg.height or not msg.width or msg.step < row_width:
        raise ValueError("invalid rectified image dimensions or stride")
    if len(msg.data) != msg.height * msg.step:
        raise ValueError("invalid rectified image data length")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :row_width].reshape(msg.height, msg.width, channels)
    if msg.encoding == "mono8":
        return np.repeat(pixels, 3, axis=2)
    return pixels[:, :, ::-1] if msg.encoding == "bgr8" else pixels


def _sharpness(rgb: np.ndarray) -> float:
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 3:
        raise ValueError("expected an RGB8 image at least 3 pixels wide and tall")
    gray = rgb @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
    laplacian = (
        gray[:-2, 1:-1] + gray[2:, 1:-1] + gray[1:-1, :-2] + gray[1:-1, 2:]
        - 4 * gray[1:-1, 1:-1]
    )
    return float(laplacian.var())


class PhotoLog:
    """
    Saves captured photos into a per-run directory, indexed in a JSONL file.
    """

    INDEX_FILENAME = "photos.jsonl"

    def __init__(self, data_dir: Path, now: datetime) -> None:
        self._run_dir = data_dir / f"run_{now.strftime('%Y-%m-%dT%H-%M-%S')}"
        self._run_dir.mkdir(parents=True, exist_ok=True)
        self._index_file = self._run_dir / self.INDEX_FILENAME
        self._count = 0

    @property
    def run_dir(self) -> Path:
        return self._run_dir

    @property
    def index_file(self) -> Path:
        return self._index_file

    def save(
        self,
        rgb: np.ndarray,
        setpoint: CoordinateMsg,
        position: CoordinateMsg,
        stamp: str,
        metadata: dict | None = None,
    ) -> Path:
        self._count += 1
        photo_file = self._run_dir / f"photo_{self._count:04d}.png"
        PILImage.fromarray(rgb).save(photo_file)

        record = {
            "stamp": stamp,
            "file": photo_file.name,
            "setpoint": _coordinate_record(setpoint),
            "position": _coordinate_record(position),
        }
        if metadata is not None:
            record.update(metadata)
        with self._index_file.open("a") as f:
            f.write(json.dumps(record) + "\n")
        return photo_file


class CameraNode(Node):
    CAPTURE_SERVICE = "camera/capture"
    DEFAULT_DATA_DIR = "/ros_ws/data/photos"

    def __init__(self) -> None:
        super().__init__("camera_node")

        topic = self.declare_parameter(
            "rectified_image_topic", os.environ.get("RECTIFIED_IMAGE_TOPIC", "")
        ).value
        self._min_sharpness = self.declare_parameter("min_sharpness", 100.0).value
        self._max_frame_age_s = self.declare_parameter("max_frame_age_s", 0.5).value
        if not math.isfinite(self._min_sharpness) or self._min_sharpness <= 0:
            raise ValueError("min_sharpness must be positive and finite")
        if not math.isfinite(self._max_frame_age_s) or self._max_frame_age_s <= 0:
            raise ValueError("max_frame_age_s must be positive and finite")
        self._camera: AbstractCamera | None = None
        self._latest_rectified: tuple[Image, int] | None = None
        if topic:
            self._rectified_sub = self.create_subscription(
                Image, topic, self._on_rectified_image, qos_profile_sensor_data
            )
        else:
            self._camera = SimCamera()
            if not self._camera.initialize_camera():
                raise RuntimeError("could not initialize the camera")

        data_dir = Path(os.environ.get("PHOTO_DATA_DIR", self.DEFAULT_DATA_DIR))
        self._photo_log = PhotoLog(data_dir, datetime.now())

        self._capture_service = self.create_service(
            CapturePhoto, self.CAPTURE_SERVICE, self._on_capture
        )

        self.get_logger().info(
            f"Camera node ready - source: {topic or 'in-process camera'}, saving captures to "
            f"'{self._photo_log.run_dir}'."
        )

    def _on_rectified_image(self, msg: Image) -> None:
        stamp_ns = _stamp_ns(msg.header.stamp) or self.get_clock().now().nanoseconds
        self._latest_rectified = (msg, stamp_ns)

    def _frame_for_capture(self) -> tuple[np.ndarray, int]:
        if self._camera is not None:
            frame = self._camera.capture_frame()
            if frame is None:
                raise ValueError("no camera frame available")
            return frame.rgb, self.get_clock().now().nanoseconds
        if self._latest_rectified is None:
            raise ValueError("no rectified camera frame available yet")
        msg, stamp_ns = self._latest_rectified
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not 0 <= age_s <= self._max_frame_age_s:
            raise ValueError("stale or future-dated rectified camera frame")
        return _rectified_rgb(msg), stamp_ns

    def _on_capture(
        self, request: CapturePhoto.Request, response: CapturePhoto.Response
    ) -> CapturePhoto.Response:
        try:
            rgb, frame_stamp_ns = self._frame_for_capture()
            sharpness = _sharpness(rgb)
            if sharpness < self._min_sharpness:
                raise ValueError(
                    f"sharpness {sharpness:.1f} below threshold {self._min_sharpness:.1f}"
                )
            photo_file = self._photo_log.save(
                rgb,
                setpoint=request.setpoint,
                position=request.position,
                stamp=datetime.now().isoformat(),
                metadata={
                    "frame_stamp_ns": frame_stamp_ns,
                    "sharpness": sharpness,
                    "attitude": (
                        _attitude_record(request.imu, frame_stamp_ns)
                        if request.has_imu else None
                    ),
                },
            )
        except ValueError as error:
            response.success = False
            response.message = str(error)
            self.get_logger().warning(f"Capture rejected: {error}")
            return response
        except OSError as error:
            response.success = False
            response.message = f"failed to save photo: {error}"
            self.get_logger().error(f"Capture failed: {response.message}")
            return response

        response.success = True
        response.path = str(photo_file)
        response.message = "captured"
        self.get_logger().info(f"Captured '{photo_file}'")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node._camera is not None:
            node._camera.close_camera()
        node.destroy_node()
        rclpy.try_shutdown()
