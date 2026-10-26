from __future__ import annotations

import py_trees
import rclpy.node
from engine.ground_log import send_to_ground
from std_msgs.msg import Empty


class WaitForGoHome(py_trees.behaviour.Behaviour):
    """
    Waits for the go-home command from the ground.

    Returns RUNNING until a message arrives on ``mission/go_home``, then
    SUCCESS on every tick after that.
    """

    GO_HOME_TOPIC = "mission/go_home"

    def __init__(self, name: str = "WaitForGoHome") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._go_home_received = False

        self._go_home_sub = self._node.create_subscription(
            msg_type=Empty,
            topic=self.GO_HOME_TOPIC,
            callback=self._go_home_callback,
            qos_profile=10,
        )

    def _go_home_callback(self, msg: Empty) -> None:
        if self._go_home_received:
            return
        self._go_home_received = True
        self._node.get_logger().info(f"{self.name}: go-home command received")
        send_to_ground(self._node, "ENG: go-home received")

    def update(self) -> py_trees.common.Status:
        if self._go_home_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
