from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys


class LoadHomeWaypoint(py_trees.behaviour.Behaviour):
    """
    Stages the home coordinate as the next waypoint for the return flight.

    Reads ``home_waypoint`` and writes it to ``current_waypoint``.
    Returns FAILURE if no home coordinate was configured.
    """

    def __init__(self, name: str = "LoadHomeWaypoint") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.HOME_WAYPOINT, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            home = self.blackboard.get(blackboard_keys.HOME_WAYPOINT)
        except KeyError:
            home = None

        if home is None:
            self._node.get_logger().error(
                f"{self.name}: no home coordinate on the blackboard"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.CURRENT_WAYPOINT, home)
        self._node.get_logger().info(f"{self.name}: returning home to {home}")
        return py_trees.common.Status.SUCCESS
