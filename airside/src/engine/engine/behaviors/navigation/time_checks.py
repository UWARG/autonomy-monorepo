"""
Condition behaviors that ensure laps are completed before the lapping deadline.

If the deadline was never set on the blackboard both conditions always succeed.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import LAP_TIME_MARGIN


class EnoughTimeRemaining(py_trees.behaviour.Behaviour):
    """
    Succeed if the lapping deadline has not passed.
    """

    def __init__(self, name: str = "EnoughTimeRemaining") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAPPING_END_TIME_SEC, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            end_time_s = self.blackboard.get(blackboard_keys.LAPPING_END_TIME_SEC)
        except KeyError:
            return py_trees.common.Status.SUCCESS

        now_s = self._node.get_clock().now().nanoseconds / 1e9
        if now_s < end_time_s:
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warning(
            f"{self.name}: lapping deadline passed, cutting lap short"
        )
        return py_trees.common.Status.FAILURE


class EnoughTimeForAnotherLap(py_trees.behaviour.Behaviour):
    """
    Succeed if a whole lap is expected to fit before the lapping deadline
    (scaled by a safety margin).
    """

    def __init__(self, name: str = "EnoughTimeForAnotherLap") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAPPING_END_TIME_SEC, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.ESTIMATED_LAP_TIME_SEC,
            access=py_trees.common.Access.READ,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            end_time_s = self.blackboard.get(blackboard_keys.LAPPING_END_TIME_SEC)
        except KeyError:
            return py_trees.common.Status.SUCCESS

        remaining_s = end_time_s - self._node.get_clock().now().nanoseconds / 1e9

        try:
            estimated_lap_time_s = self.blackboard.get(
                blackboard_keys.ESTIMATED_LAP_TIME_SEC
            )
        except KeyError:
            estimated_lap_time_s = None

        if estimated_lap_time_s is None:
            if remaining_s > 0.0:
                return py_trees.common.Status.SUCCESS
            self._node.get_logger().warning(
                f"{self.name}: lapping deadline passed, not starting a lap"
            )
            return py_trees.common.Status.FAILURE

        required_s = estimated_lap_time_s * LAP_TIME_MARGIN
        if remaining_s >= required_s:
            self._node.get_logger().info(
                f"{self.name}: {remaining_s:.1f}s left, lap needs ~{required_s:.1f}s"
            )
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warning(
            f"{self.name}: only {remaining_s:.1f}s left but a lap needs "
            f"~{required_s:.1f}s, not starting another lap"
        )
        return py_trees.common.Status.FAILURE
