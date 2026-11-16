from __future__ import annotations

import collections
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


class CaptureRejected(Exception):
    """
    An expected reason why a photo cannot be saved.

    Missing, stale or blurry frames raise this. Any other exception is a bug
    and deliberately propagates instead of reporting a quiet rejection.
    """


# Clamps so a misconfigured parameter degrades captures instead of crashing
# the node at startup.
SHARPNESS_CEILING = 10_000.0
FRAME_AGE_CEILING_S = 5.0
IMU_OFFSET_CEILING_S = 5.0

# Recent IMU samples kept for matching against a frame (seconds of coverage
# at MAVROS's typical 50-100 Hz).
IMU_HISTORY_LIMIT = 200


def _coordinate_record(msg) -> dict[str, float]:
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


def _closest_imu_sample(
    samples: collections.deque, frame_stamp_ns: int, max_offset_ns: int
) -> tuple[int, Imu] | None:
    """
    Return the (stamp_ns, Imu) pair closest to the frame time.

    Only samples within ``max_offset_ns`` of the frame count; a stale or
    drifted IMU stream contributes nothing. Not hardware sync: this only
    bounds how far apart the two timestamps may be.
    """
    candidates = [
        pair for pair in samples if abs(pair[0] - frame_stamp_ns) <= max_offset_ns
    ]
    return min(candidates, key=lambda pair: abs(pair[0] - frame_stamp_ns), default=None)


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
    DEFAULT_IMU_TOPIC = "mavros/imu/data"

    def __init__(self) -> None:
        super().__init__("camera_node")

        topic = self.declare_parameter(
            "rectified_image_topic", os.environ.get("RECTIFIED_IMAGE_TOPIC", "")
        ).value
        self._min_sharpness = self._bounded_parameter(
            "min_sharpness",
            self.declare_parameter("min_sharpness", 100.0).value,
            SHARPNESS_CEILING,
            100.0,
        )
        self._max_frame_age_s = self._bounded_parameter(
            "max_frame_age_s",
            self.declare_parameter("max_frame_age_s", 0.5).value,
            FRAME_AGE_CEILING_S,
            0.5,
        )
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

        # Subscribe here so the logged attitude is close to the frame's own
        # timestamp, not to whenever the behaviour tree sent the request.
        # Empty imu_topic disables the association.
        imu_topic = self.declare_parameter(
            "imu_topic", os.environ.get("IMU_TOPIC", self.DEFAULT_IMU_TOPIC)
        ).value
        self._max_imu_offset_s = self._bounded_parameter(
            "max_imu_offset_s",
            self.declare_parameter("max_imu_offset_s", 0.25).value,
            IMU_OFFSET_CEILING_S,
            0.25,
        )
        self._max_imu_offset_ns = int(self._max_imu_offset_s * 1e9)
        self._imu_samples: collections.deque = collections.deque(
            maxlen=IMU_HISTORY_LIMIT
        )
        if imu_topic:
            self._imu_sub = self.create_subscription(
                Imu, imu_topic, self._on_imu, qos_profile_sensor_data
            )

        data_dir = Path(os.environ.get("PHOTO_DATA_DIR", self.DEFAULT_DATA_DIR))
        self._photo_log = PhotoLog(data_dir, datetime.now())

        self._capture_service = self.create_service(
            CapturePhoto, self.CAPTURE_SERVICE, self._on_capture
        )

        self.get_logger().info(
            f"Camera node ready - source: {topic or 'in-process camera'}, saving captures to "
            f"'{self._photo_log.run_dir}'."
        )

    def _bounded_parameter(
        self, name: str, value: object, ceiling: float, default: float
    ) -> float:
        """
        Clamp a numeric parameter into the (0, ceiling] range with a warning.

        Non-numeric, non-finite, zero or negative values fall back to
        ``default``; values above the ceiling are clamped. Never raises.
        """
        if (
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            self.get_logger().warning(
                f"parameter '{name}' = {value!r} is not a positive finite number; "
                f"using the default {default} instead"
            )
            return default
        if value > ceiling:
            self.get_logger().warning(
                f"parameter '{name}' = {value!r} is above the {ceiling} ceiling; "
                f"clamped to {ceiling}"
            )
            return ceiling
        return float(value)

    def _on_rectified_image(self, msg: Image) -> None:
        stamp_ns = _stamp_ns(msg.header.stamp) or self.get_clock().now().nanoseconds
        self._latest_rectified = (msg, stamp_ns)

    def _on_imu(self, msg: Imu) -> None:
        stamp_ns = _stamp_ns(msg.header.stamp)
        if not stamp_ns:
            # An unstamped sample cannot be matched to any frame.
            return
        self._imu_samples.append((stamp_ns, msg))

    def _frame_for_capture(self) -> tuple[np.ndarray, int]:
        if self._camera is not None:
            frame = self._camera.capture_frame()
            if frame is None:
                raise CaptureRejected("no camera frame available")
            return frame.rgb, self.get_clock().now().nanoseconds
        if self._latest_rectified is None:
            raise CaptureRejected("no rectified camera frame available yet")
        msg, stamp_ns = self._latest_rectified
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not 0 <= age_s <= self._max_frame_age_s:
            raise CaptureRejected("stale or future-dated rectified camera frame")
        return _rectified_rgb(msg), stamp_ns

    def _attitude_for_frame(self, frame_stamp_ns: int) -> dict | None:
        """
        Return the IMU sample closest to the frame, or None when none is close.

        A sample further than ``max_imu_offset_s`` from the frame is treated
        as stale (MAVROS stopped publishing, clock drift); the photo is then
        logged without attitude.
        """
        closest = _closest_imu_sample(
            self._imu_samples, frame_stamp_ns, self._max_imu_offset_ns
        )
        if closest is None:
            self.get_logger().warning(
                f"no IMU sample within {self._max_imu_offset_s:.2f}s of the frame; "
                "logging the photo without attitude"
            )
            return None
        stamp_ns, imu = closest
        return _attitude_record(imu, frame_stamp_ns)

    def _on_capture(
        self, request: CapturePhoto.Request, response: CapturePhoto.Response
    ) -> CapturePhoto.Response:
        """
        Save a photo when the frame and its metadata are usable.

        Only rejections (``CaptureRejected``) and storage errors
        (``OSError``) are caught; malformed image data or bugs propagate so
        they show up as tracebacks, not quiet "rejected" responses.
        """
        try:
            rgb, frame_stamp_ns = self._frame_for_capture()
            sharpness = _sharpness(rgb)
            if sharpness < self._min_sharpness:
                raise CaptureRejected(
                    f"sharpness {sharpness:.1f} below threshold {self._min_sharpness:.1f}"
                )
            attitude = self._attitude_for_frame(frame_stamp_ns)
            photo_file = self._photo_log.save(
                rgb,
                setpoint=request.setpoint,
                position=request.position,
                stamp=datetime.now().isoformat(),
                metadata={
                    "frame_stamp_ns": frame_stamp_ns,
                    "sharpness": sharpness,
                    "attitude": attitude,
                },
            )
        except CaptureRejected as error:
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
