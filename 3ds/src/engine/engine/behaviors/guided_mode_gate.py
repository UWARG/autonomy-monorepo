"""
Decorator that only lets the mission run while the pilot has handed over
control in GUIDED mode.
"""

from __future__ import annotations

import typing

import py_trees
import rclpy.node
from engine.constants import GUIDED_MODE, LAND_MODE
from engine.ground_log import send_to_ground
from mavros_msgs.msg import State

_STATE_TOPIC = "mavros/state"

# LAND is let through so the Land behavior can see the touchdown it commanded.
_AUTONOMY_MODES = (GUIDED_MODE, LAND_MODE)


class PauseUnlessGuided(py_trees.decorators.Decorator):
    """
    Pauses the decorated subtree unless the flight controller is in GUIDED.

    The engine never changes flight mode itself: the pilot selects GUIDED to
    hand over control and any other mode to take it back. While paused the
    child is not ticked, so nothing is commanded and its progress is kept;
    switching back to GUIDED resumes the mission where it left off.
    """

    def __init__(
        self, child: py_trees.behaviour.Behaviour, name: str = "PauseUnlessGuided"
    ) -> None:
        super().__init__(name=name, child=child)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None
        self._paused = False

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=_STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def tick(self) -> typing.Iterator[py_trees.behaviour.Behaviour]:
        mode = self._latest_state.mode if self._latest_state is not None else None
        if mode not in _AUTONOMY_MODES:
            if not self._paused:
                self._paused = True
                send_to_ground(self._node, f"ENG: paused, waiting for {GUIDED_MODE}")
            self._node.get_logger().warning(
                f"{self.name}: flight controller in '{mode}' mode, waiting for "
                f"the pilot to select '{GUIDED_MODE}'",
                throttle_duration_sec=5.0,
            )
            self.feedback_message = f"paused, waiting for {GUIDED_MODE}"
            self.status = py_trees.common.Status.RUNNING
            yield self
            return

        if self._paused:
            self._paused = False
            self.feedback_message = ""
            send_to_ground(self._node, "ENG: resumed")
            self._node.get_logger().info(f"{self.name}: '{mode}' mode, resuming")

        yield from super().tick()

    def update(self) -> py_trees.common.Status:
        return self.decorated.status
