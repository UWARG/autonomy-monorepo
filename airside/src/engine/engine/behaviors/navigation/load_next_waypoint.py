from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys
from utils.src.types import Coordinate


class LoadNextWaypoint(py_trees.behaviour.Behaviour):
    """
    Advances to the next waypoint in the current lap.

    Reads ``waypoints`` and ``waypoint_index`` from the blackboard, writes the
    waypoint at the index to ``current_waypoint``, and increments the index.
    Also writes the waypoint after it to ``following_waypoint``.
    Returns SUCCESS when a waypoint was loaded, FAILURE when the list is
    exhausted.
    """

    def __init__(self, name: str = "LoadNextWaypoint") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINTS, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=blackboard_keys.FOLLOWING_WAYPOINT, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=blackboard_keys.LAPS_REMAINING, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            waypoints = self.blackboard.get(blackboard_keys.WAYPOINTS)
            index = self.blackboard.get(blackboard_keys.WAYPOINT_INDEX)
        except KeyError:
            self._node.get_logger().error(
                f"{self.name}: no waypoint list on the blackboard"
            )
            return py_trees.common.Status.FAILURE

        if index >= len(waypoints):
            self._node.get_logger().info(
                f"{self.name}: all {len(waypoints)} waypoints visited, lap complete"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.CURRENT_WAYPOINT, waypoints[index])
        self.blackboard.set(
            blackboard_keys.FOLLOWING_WAYPOINT, self._following(waypoints, index)
        )
        self.blackboard.set(blackboard_keys.WAYPOINT_INDEX, index + 1)
        self._node.get_logger().info(
            f"{self.name}: waypoint {index + 1}/{len(waypoints)}: {waypoints[index]}"
        )
        return py_trees.common.Status.SUCCESS

    def _following(
        self, waypoints: list[Coordinate], index: int
    ) -> Coordinate | None:
        """The waypoint flown to after ``waypoints[index]``, or None."""

        if index + 1 < len(waypoints):
            return waypoints[index + 1]
        try:
            laps_remaining = self.blackboard.get(blackboard_keys.LAPS_REMAINING)
        except KeyError:
            return None
        return waypoints[0] if laps_remaining > 0 else None
