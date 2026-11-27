"""
Behaviors that limit lapping to a fixed number of laps.

If the lap count was never set on the blackboard, laps continue indefinitely.
"""

from __future__ import annotations

import py_trees
import rclpy.node

from engine import blackboard_keys
from engine.constants import LAP_COUNT
from engine.ground_log import send_to_ground

_LAP_COUNT_PARAMETER = "lap_count"


class SetLapCount(py_trees.behaviour.Behaviour):
    """
    Writes the number of laps to fly to the blackboard when lapping begins.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "SetLapCount") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAPS_REMAINING,
            access=py_trees.common.Access.WRITE,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._node.declare_parameter(_LAP_COUNT_PARAMETER, LAP_COUNT)

    def update(self) -> py_trees.common.Status:
        lap_count = int(self._node.get_parameter(_LAP_COUNT_PARAMETER).value)
        self.blackboard.set(blackboard_keys.LAPS_REMAINING, lap_count)
        self._node.get_logger().info(f"{self.name}: flying {lap_count} laps")
        send_to_ground(self._node, f"ENG: lapping phase, {lap_count} laps")
        return py_trees.common.Status.SUCCESS


class AnotherLapRemaining(py_trees.behaviour.Behaviour):
    """
    Succeed and consume one lap if any laps remain, fail otherwise.

    Laps cut short by a waypoint timeout still count towards the total.
    """

    def __init__(self, name: str = "AnotherLapRemaining") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAPS_REMAINING,
            access=py_trees.common.Access.WRITE,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            laps_remaining = self.blackboard.get(blackboard_keys.LAPS_REMAINING)
        except KeyError:
            return py_trees.common.Status.SUCCESS

        if laps_remaining <= 0:
            self._node.get_logger().info(f"{self.name}: all laps flown")
            send_to_ground(self._node, "ENG: lapping done, all laps flown")
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.LAPS_REMAINING, laps_remaining - 1)
        self._node.get_logger().info(
            f"{self.name}: starting lap, {laps_remaining - 1} left after this"
        )
        return py_trees.common.Status.SUCCESS
