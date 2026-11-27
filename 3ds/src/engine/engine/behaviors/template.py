"""
To create a new behavior, copy this file, rename it, and fill in the lifecycle methods.
"""

from __future__ import annotations

import py_trees
import rclpy.node


class TemplateBehavior(py_trees.behaviour.Behaviour):
    """
    A template behavior.
    """

    def __init__(self, name: str = "TemplateBehavior") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)

        # To read:
        # self.blackboard.register_key(key="altitude", access=py_trees.common.Access.READ)

        # To write:
        # self.blackboard.register_key(key="waypoint", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        """
        Called once during tree.setup().
        """

        self._node = kwargs["node"]

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """

        pass

    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """
        Called when the behavior exits for any reason.
        """

        pass
