"""
Payload release behavior.

The drop mechanism is a servo actuated over MAVROS: the behavior sends a
``MAV_CMD_DO_SET_SERVO`` command via the ``mavros/cmd/command`` service to move
the release servo to its open position. The servo channel and PWM values live
in ``engine.constants`` and still need to be confirmed against the drop
hardware.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import (
    PAYLOAD_RELEASE_OPEN_PWM,
    PAYLOAD_RELEASE_SERVO_CHANNEL,
)
from engine.ground_log import send_to_ground
from mavros_msgs.srv import CommandLong

_COMMAND_SERVICE = "mavros/cmd/command"

# MAVLink command to set a servo output to a PWM value.
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO
_MAV_CMD_DO_SET_SERVO = 183


class ReleasePayload(py_trees.behaviour.Behaviour):
    """
    Releases one payload item by driving the release servo open over MAVROS.

    Sends a ``MAV_CMD_DO_SET_SERVO`` command to move servo channel
    ``PAYLOAD_RELEASE_SERVO_CHANNEL`` to ``PAYLOAD_RELEASE_OPEN_PWM``. On a
    successful command ``items_remaining`` is decremented and SUCCESS is
    returned. Returns RUNNING until the service responds and FAILURE if the
    command is rejected.
    """

    def __init__(self, name: str = "ReleasePayload") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.ITEMS_REMAINING, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._command_future = None

        self._command_client = self._node.create_client(
            srv_type=CommandLong, srv_name=_COMMAND_SERVICE
        )

    def initialise(self) -> None:
        self._command_future = None

    def update(self) -> py_trees.common.Status:
        if self._command_future is None:
            if not self._command_client.service_is_ready():
                self._node.get_logger().warning(
                    f"{self.name}: waiting for '{_COMMAND_SERVICE}' service",
                    throttle_duration_sec=5.0,
                )
                return py_trees.common.Status.RUNNING

            request = CommandLong.Request()
            request.command = _MAV_CMD_DO_SET_SERVO
            request.param1 = float(PAYLOAD_RELEASE_SERVO_CHANNEL)
            request.param2 = float(PAYLOAD_RELEASE_OPEN_PWM)
            self._command_future = self._command_client.call_async(request)
            self._node.get_logger().info(
                f"{self.name}: opening release servo (channel "
                f"{PAYLOAD_RELEASE_SERVO_CHANNEL}, PWM {PAYLOAD_RELEASE_OPEN_PWM})"
            )
            return py_trees.common.Status.RUNNING

        if not self._command_future.done():
            return py_trees.common.Status.RUNNING

        response = self._command_future.result()
        self._command_future = None
        if response is None or not response.success:
            self._node.get_logger().error(
                f"{self.name}: servo release command rejected: {response}"
            )
            return py_trees.common.Status.FAILURE

        try:
            items_remaining = self.blackboard.get(blackboard_keys.ITEMS_REMAINING)
        except KeyError:
            items_remaining = 0
        items_remaining = max(items_remaining - 1, 0)
        self.blackboard.set(blackboard_keys.ITEMS_REMAINING, items_remaining)

        self._node.get_logger().info(
            f"{self.name}: payload released, {items_remaining} items left"
        )
        send_to_ground(self._node, f"ENG: payload dropped, {items_remaining} left")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._command_future = None
