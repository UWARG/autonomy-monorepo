"""
Condition behavior confirming the drone has landed on the pad before its
payload is released.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine.ground_log import send_to_ground
from mavros_msgs.msg import State

_STATE_TOPIC = "mavros/state"


class TouchingGround(py_trees.behaviour.Behaviour):
    """
    Blocks until the drone has landed on the pad, then succeeds.

    Subscribes to ``mavros/state`` and returns RUNNING until the flight
    controller disarms after the LAND-mode descent, then SUCCESS. A disarmed
    flight controller is how the ``Land`` behavior detects touchdown, and it
    satisfies the competition rule that a payload is only released once the UAV
    has landed.
    """

    def __init__(self, name: str = "TouchingGround") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=_STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def update(self) -> py_trees.common.Status:
        if self._latest_state is None:
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{_STATE_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._latest_state.armed:
            self._node.get_logger().info(f"{self.name}: landed on pad")
            send_to_ground(self._node, "ENG: on the ground, ready to drop")
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: waiting for touchdown",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING
