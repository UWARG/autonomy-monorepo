from __future__ import annotations

from typing import Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

from camera.src.abstract_camera import AbstractCamera
from camera.src.arducam import Arducam
from camera.src.oakd import OakD
from camera.src.sim import SimCamera

CAMERA_TYPES: dict[str, Callable[[], AbstractCamera]] = {
    "sim": SimCamera,
    "oakd": OakD,
    "arducam": Arducam,
}


class CameraNode(Node):
    TOPIC = "camera/image_raw"
    IMU_TOPIC = "camera/imu"
    PUBLISH_HZ = 50.0
    IMU_PUBLISH_HZ = 100.0
    WIDTH = 640
    HEIGHT = 480

    def __init__(
        self,
        topic: str = TOPIC,
        node_name: str = "camera_node",
        default_camera_type: str = "sim",
    ) -> None:
        super().__init__(node_name)

        self._topic = topic
        self.declare_parameter("camera_type", default_camera_type)
        camera_type = self.get_parameter("camera_type").get_parameter_value().string_value
        camera_factory = CAMERA_TYPES.get(camera_type)
        if camera_factory is None:
            self.get_logger().warning(
                f"Unknown camera_type '{camera_type}', falling back to 'sim'. "
                f"Valid options: {', '.join(CAMERA_TYPES)}"
            )
            camera_factory = SimCamera

        self._camera: AbstractCamera = camera_factory()
        if not self._camera.initialize_camera():
            self.get_logger().error(
                f"Failed to initialize {type(self._camera).__name__}; publishing black frames."
            )

        self._publisher = self.create_publisher(Image, self._topic, 10)
        self._frame_id = 0
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish_frame)

        if isinstance(self._camera, OakD):
            self._imu_publisher = self.create_publisher(Imu, self.IMU_TOPIC, 10)
            self.create_timer(1.0 / self.IMU_PUBLISH_HZ, self._publish_imu)
        else:
            self._imu_publisher = None

        self.get_logger().info(
            f"Camera node ready - publishing on '{self._topic}' at {self.PUBLISH_HZ} Hz "
            f"using {type(self._camera).__name__}."
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
            msg.data.frombytes(bytes(self.HEIGHT * self.WIDTH * 3))
        else:
            msg.height = frame.rgb.shape[0]
            msg.width = frame.rgb.shape[1]
            msg.step = frame.rgb.shape[1] * 3
            msg.data.frombytes(frame.rgb.tobytes())

        self._publisher.publish(msg)
        self._frame_id += 1
        self.get_logger().debug(f"Published frame {self._frame_id}")

    def _publish_imu(self) -> None:
        imu_data = self._camera.capture_imu()
        if imu_data is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        # orientation is not provided by the camera IMU
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = imu_data.gyro_x
        msg.angular_velocity.y = imu_data.gyro_y
        msg.angular_velocity.z = imu_data.gyro_z

        msg.linear_acceleration.x = imu_data.accel_x
        msg.linear_acceleration.y = imu_data.accel_y
        msg.linear_acceleration.z = imu_data.accel_z

        self._imu_publisher.publish(msg)


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
