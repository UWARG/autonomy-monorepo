"""
Behaviors that react to RC transmitter switches via ``mavros/rc/in``.
"""

from __future__ import annotations

import typing

import py_trees
import rclpy.node
from engine.constants import KILL_SWITCH_RC_CHANNEL, RC_SWITCH_HIGH_PWM
from mavros_msgs.msg import RCIn
from rclpy.qos import qos_profile_sensor_data


class RCChannelListener:
    """
    Mixin that tracks whether an RC switch channel is high.
    """

    RC_IN_TOPIC = "mavros/rc/in"

    _channel: int
    name: str

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._switch_high: bool | None = None  # None until RC data arrives

        self._rc_sub = self._node.create_subscription(
            msg_type=RCIn,
            topic=self.RC_IN_TOPIC,
            callback=self._rc_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def _rc_callback(self, msg: RCIn) -> None:
        index = self._channel - 1
        if index >= len(msg.channels):
            self._node.get_logger().warning(
                f"{self.name}: RC channel {self._channel} not present in "
                f"'{self.RC_IN_TOPIC}' ({len(msg.channels)} channels)",
                throttle_duration_sec=5.0,
            )
            return
        self._switch_high = msg.channels[index] >= RC_SWITCH_HIGH_PWM


class RCSwitchMonitor(RCChannelListener, py_trees.behaviour.Behaviour):
    """
    Base class that tracks whether an RC switch channel is high.
    """

    def __init__(self, name: str, channel: int) -> None:
        super().__init__(name=name)
        self._channel = channel


class KillSwitch(RCChannelListener, py_trees.decorators.Decorator):
    """
    Pauses the decorated subtree while the kill switch is high.
    """

    def __init__(
        self,
        child: py_trees.behaviour.Behaviour,
        name: str = "KillSwitch",
        channel: int = KILL_SWITCH_RC_CHANNEL,
    ) -> None:
        super().__init__(name=name, child=child)
        self._channel = channel

    def tick(self) -> typing.Iterator[py_trees.behaviour.Behaviour]:
        if self._switch_high:
            self._node.get_logger().warning(
                f"{self.name}: RC channel {self._channel} flipped, pausing mission",
                throttle_duration_sec=5.0,
            )
            self.feedback_message = "mission paused by kill switch"
            self.status = py_trees.common.Status.RUNNING
            yield self
            return

        yield from super().tick()

    def update(self) -> py_trees.common.Status:
        return self.decorated.status


class WaitForRCSwitch(RCSwitchMonitor):
    """
    Waits until the monitored RC switch is flipped.

    Returns RUNNING until the channel reads high, then SUCCESS.
    """

    def update(self) -> py_trees.common.Status:
        if self._switch_high is None:
            self._node.get_logger().warning(
                f"{self.name}: waiting for RC data on '{self.RC_IN_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if self._switch_high:
            self._node.get_logger().info(
                f"{self.name}: RC channel {self._channel} flipped"
            )
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
