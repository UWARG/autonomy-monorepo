"""
To create a new behavior, copy this file, rename it, and fill in the lifecycle methods.
"""

from __future__ import annotations

import py_trees
import rclpy.node

from mavros_msgs.msg import RCIn

class FlyAround(py_trees.behaviour.Behaviour):
    """
    Fly around the area.
    """

    def __init__(self) -> None:
        super().__init__(name="FlyAround")


    def setup(self, **kwargs: rclpy.node.Node) -> None:
        """
        Called once during tree.setup().
        """

        self._node = kwargs["node"]
        self.rc_subscriber=self._node.create_subscription(RCIn, "/rc", self.rc_callback, 10)

    def rc_callback(self, msg: RCIn):
        self.channel6=msg.channels[6]

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self.channel6=1500

    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        if self.channel6>1600:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
