from __future__ import annotations

import py_trees
import rclpy.node
from sensor_msgs.msg import Image


class ReadCameraBehavior(py_trees.behaviour.Behaviour):
    """
    A behavior that reads a single frame from the camera topic.

    Subscribes to ``camera/image_raw``, stores the latest ``Image`` message on
    the blackboard under the key ``camera/image``, then returns SUCCESS once a
    frame has been received.  Returns RUNNING while waiting for the first frame.
    """

    TOPIC = "camera/image_raw"

    def __init__(self, name: str = "ReadCameraBehavior") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="camera/image", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_image: Image | None = None

        self._sub = self._node.create_subscription(
            msg_type=Image,
            topic=self.TOPIC,
            callback=self._camera_callback,
            qos_profile=10,
        )
        self._node.get_logger().info(
            f"ReadCameraBehavior: subscribed to '{self.TOPIC}'"
        )

    def _camera_callback(self, msg: Image) -> None:
        self._latest_image = msg

    def initialise(self) -> None:
        """Reset cached frame."""
        self._latest_image = None
        self._node.get_logger().info("ReadCameraBehavior: initialise - waiting for frame")

    def update(self) -> py_trees.common.Status:
        """Write the latest frame to the blackboard and return SUCCESS."""
        if self._latest_image is None:
            self._node.get_logger().info("ReadCameraBehavior: update - RUNNING (no frame yet)")
            return py_trees.common.Status.RUNNING

        self.blackboard.set("camera/image", self._latest_image)
        self._node.get_logger().info(
            f"ReadCameraBehavior: update - SUCCESS "
            f"({self._latest_image.width}x{self._latest_image.height}, "
            f"{len(self._latest_image.data)} bytes)"
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._node.get_logger().info(
            f"ReadCameraBehavior: terminate - exiting with {new_status}"
        )
