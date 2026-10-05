from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys


class LoadLaunchPoint(py_trees.behaviour.Behaviour):
    """
    Stages the launch point as the next waypoint for the return flight.

    Reads ``launch_point`` and writes it to ``current_waypoint``.
    Returns FAILURE if no launch point was recorded.
    """

    def __init__(self, name: str = "LoadLaunchPoint") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAUNCH_POINT, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            launch_point = self.blackboard.get(blackboard_keys.LAUNCH_POINT)
        except KeyError:
            self._node.get_logger().error(
                f"{self.name}: no launch point on the blackboard"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.CURRENT_WAYPOINT, launch_point)
        self._node.get_logger().info(
            f"{self.name}: returning to launch point {launch_point}"
        )
        return py_trees.common.Status.SUCCESS
