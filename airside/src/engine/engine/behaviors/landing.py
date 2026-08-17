"""
To create a new behavior, copy this file, rename it, and fill in the lifecycle methods.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from rclpy.action import ActionClient
from custom_interfaces.action import Landing as LandingAction
from mavros_msgs.msg import State
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
        self._landing_action_client=ActionClient(self._node, LandingAction, "/landing")
        self.state_subscriber=self._node.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.state="GUIDED"
        self._goal_handle=None

    def state_callback(self, msg: State):
        self.state=msg.mode

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self.success=None
        while not self._landing_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().info("Waiting for landing action server")
        goal=LandingAction.Goal()
        self.goal_future=self._landing_action_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self.success=False
            return
        self._goal_handle=goal_handle
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        response=future.result()
        self.success=response.result.success

    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        if self.state != "GUIDED" and self.state != "LAND":
            self._node.get_logger().info("Pilot override")
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            return py_trees.common.Status.SUCCESS
        if self.success is None:
            return py_trees.common.Status.RUNNING
        if not self.success:
            self._node.get_logger().info("Landing Failed")
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
