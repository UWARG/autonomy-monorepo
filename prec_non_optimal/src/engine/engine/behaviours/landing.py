"""
To create a new behavior, copy this file, rename it, and fill in the lifecycle methods.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from rclpy.action import ActionClient
from mavros_msgs.action import LandingAction
from action_msgs.msg import GoalStatus

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

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self.status=None
        while not self._landing_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().info("Waiting for landing action server")
        goal=LandingAction.Goal()
        self.goal_future=self._landing_action_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected')
            self.status=GoalStatus.STATUS_ABORTED
            return
        self._node.get_logger().info('Goal accepted')
        self._goal_handle=goal_handle
        self._result_future=goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        response=future.result()
        self.status=response.status
        
    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        if self.status==GoalStatus.STATUS_ABORTED:
            self._node.get_logger().info("Landing Failed")
            return py_trees.common.Status.FAILURE
        elif self.status==GoalStatus.STATUS_CANCELED:
            self._node.get_logger().info("Landing Cancelled")
            return py_trees.common.Status.FAILURE
        if self.status==GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info("Landing Succeeded")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING