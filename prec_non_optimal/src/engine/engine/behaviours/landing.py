"""
To create a new behavior, copy this file, rename it, and fill in the lifecycle methods.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from custom_interfaces.srv import Landing

class Landing(py_trees.behaviour.Behaviour):
    """
    Land the drone.
    """

    def __init__(self) -> None:
        super().__init__(name="Landing")

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        """
        Called once during tree.setup().
        """

        self._node = kwargs["node"]
        self._landing_client=self._node.create_client(Landing, "/landing")

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self.success=None
        while not self._landing_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().info("Waiting for landing service")
        self.request=Landing.Request()
        self.future=self._landing_client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        response=future.result()
        self.success=response.success
        
    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        if not self.future.done() or self.success is None:
            return py_trees.common.Status.RUNNING
        if not self.success:
            self._node.get_logger().info("Landing Failed")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS