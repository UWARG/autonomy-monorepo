from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

from camera.src.abstract_camera import AbstractCamera

CAMERA_TYPES = ("sim", "arducam", "oakd")


def _normalize_camera_type(camera_type: str) -> str:
    return camera_type.strip().lower().replace("-", "")


def _create_camera(camera_type: str) -> AbstractCamera:
    """Build a camera backend by name.

    Hardware backends are imported lazily so the sim camera works without
    their dependencies (cv2, depthai) installed.
    """
    normalized = _normalize_camera_type(camera_type)

    if normalized == "sim":
        from camera.src.sim import SimCamera

        return SimCamera()
    if normalized == "arducam":
        from camera.src.arducam import Arducam

        return Arducam()
    if normalized == "oakd":
        from camera.src.oakd import OakD

        return OakD()

    raise ValueError(
        f"Unknown camera type '{camera_type}'. Expected one of: {', '.join(CAMERA_TYPES)}."
    )


class CameraNode(Node):
    TOPIC = "camera/image_raw"
    IMU_TOPIC = "camera/imu"
    PUBLISH_HZ = 50.0
    IMU_PUBLISH_HZ = 100.0
    WIDTH = 640
    HEIGHT = 480
    CAMERA_ENV_VAR = "CAMERA_TYPE"
    DEFAULT_CAMERA = "sim"

    def __init__(self) -> None:
        super().__init__("camera_node")

        camera_type = os.environ.get(self.CAMERA_ENV_VAR, self.DEFAULT_CAMERA)
        self._camera: AbstractCamera = _create_camera(camera_type)
        if not self._camera.initialize_camera():
            self.get_logger().error(
                f"Failed to initialize {type(self._camera).__name__}; publishing blank frames."
            )

        self._publisher = self.create_publisher(Image, self.TOPIC, 10)
        self._frame_id = 0
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)

        # Only the OAK-D exposes an IMU.
        self._imu_publisher = None
        if _normalize_camera_type(camera_type) == "oakd":
            self._imu_publisher = self.create_publisher(Imu, self.IMU_TOPIC, 10)
            self.create_timer(1.0 / self.IMU_PUBLISH_HZ, self._publish_imu)

        imu_info = (
            f" IMU on '{self.IMU_TOPIC}' at {self.IMU_PUBLISH_HZ} Hz."
            if self._imu_publisher is not None
            else ""
        )
        self.get_logger().info(
            f"Camera node ready - publishing on '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"using {type(self._camera).__name__}.{imu_info}"
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
            msg.height = frame.rgb.shape[0]
            msg.width = frame.rgb.shape[1]
            msg.step = frame.rgb.shape[1] * 3
            msg.data = frame.rgb.tobytes()

        self._publisher.publish(msg)
        self._frame_id += 1
        self.get_logger().debug(f"Published frame {self._frame_id}")

    def _publish_imu(self) -> None:
        imu = self._camera.capture_imu()
        # Skip rather than publish zeros, which would read as a valid free-fall sample.
        if imu is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.linear_acceleration.x = imu.accel_x
        msg.linear_acceleration.y = imu.accel_y
        msg.linear_acceleration.z = imu.accel_z
        msg.angular_velocity.x = imu.gyro_x
        msg.angular_velocity.y = imu.gyro_y
        msg.angular_velocity.z = imu.gyro_z
        # -1 marks orientation as unavailable; raw sensors provide no quaternion.
        msg.orientation_covariance[0] = -1.0

        self._imu_publisher.publish(msg)
        self.get_logger().debug("Published IMU sample")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
