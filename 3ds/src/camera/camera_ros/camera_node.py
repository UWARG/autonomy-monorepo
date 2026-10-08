from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.srv import CapturePhoto
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image

from camera.src.abstract_camera import AbstractCamera
from camera.src.frame import CameraFrame
from camera.src.sim import SimCamera


def _coordinate_record(msg: CoordinateMsg) -> dict[str, float]:
    return {"lat": msg.lat, "lon": msg.lon, "alt": msg.alt}


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
        with self._index_file.open("a") as f:
            f.write(json.dumps(record) + "\n")
        return photo_file


class CameraNode(Node):
    TOPIC = "camera/image_raw"
    CAPTURE_SERVICE = "camera/capture"
    PUBLISH_HZ = 50.0
    WIDTH = 640
    HEIGHT = 480
    DEFAULT_DATA_DIR = "/ros_ws/data/photos"

    def __init__(self) -> None:
        super().__init__("camera_node")

        self._camera: AbstractCamera = SimCamera()
        self._camera.initialize_camera()
        self._latest_frame: CameraFrame | None = None

        data_dir = Path(os.environ.get("PHOTO_DATA_DIR", self.DEFAULT_DATA_DIR))
        self._photo_log = PhotoLog(data_dir, datetime.now())

        self._publisher = self.create_publisher(Image, self.TOPIC, 10)
        self._frame_id = 0
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)

        self._capture_service = self.create_service(
            CapturePhoto, self.CAPTURE_SERVICE, self._on_capture
        )

        self.get_logger().info(
            f"Camera node ready - publishing on '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"using {type(self._camera).__name__}, saving captures to "
            f"'{self._photo_log.run_dir}'."
        )

    def _publish_frame(self) -> None:
        frame = self._camera.capture_frame()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.encoding = "rgb8"

        if frame is None:
            msg.height = self.HEIGHT
            msg.width = self.WIDTH
            msg.step = self.WIDTH * 3
            msg.data = bytes(self.HEIGHT * self.WIDTH * 3)
        else:
            self._latest_frame = frame
            msg.height = frame.rgb.shape[0]
            msg.width = frame.rgb.shape[1]
            msg.step = frame.rgb.shape[1] * 3
            msg.data = frame.rgb.tobytes()

        self._publisher.publish(msg)
        self._frame_id += 1
        self.get_logger().debug(f"Published frame {self._frame_id}")

    def _on_capture(
        self, request: CapturePhoto.Request, response: CapturePhoto.Response
    ) -> CapturePhoto.Response:
        if self._latest_frame is None:
            response.success = False
            response.message = "no camera frame available yet"
            self.get_logger().error(f"Capture failed: {response.message}")
            return response

        try:
            photo_file = self._photo_log.save(
                self._latest_frame.rgb,
                setpoint=request.setpoint,
                position=request.position,
                stamp=datetime.now().isoformat(),
            )
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
        node.destroy_node()
        rclpy.try_shutdown()
