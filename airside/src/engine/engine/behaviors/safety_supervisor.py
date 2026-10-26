"""
Safety supervisor behaviors that react to commands from the ground.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine.ground_log import send_to_ground
from std_msgs.msg import Empty


class CheckAbort(py_trees.behaviour.Behaviour):
    """
    Checks whether an ABORT command has been received from the ground.

    Returns FAILURE until a message arrives on ``/commander/abort``, then
    SUCCESS on every tick after that.
    """

    ABORT_TOPIC = "/commander/abort"

    def __init__(self, name: str = "CheckAbort") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._abort_received = False

        self._abort_sub = self._node.create_subscription(
            msg_type=Empty,
            topic=self.ABORT_TOPIC,
            callback=self._abort_callback,
            qos_profile=10,
        )

    def _abort_callback(self, msg: Empty) -> None:
        if self._abort_received:
            return
        self._abort_received = True
        self._node.get_logger().error(f"{self.name}: ABORT received")
        send_to_ground(self._node, "ENG: ABORT received")

    def update(self) -> py_trees.common.Status:
        if self._abort_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE