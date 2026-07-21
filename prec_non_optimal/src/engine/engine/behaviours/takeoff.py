"""
To create a new behavior, copy this file, rename it, and fill in the lifecycle methods.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from mavros_msgs.msg import RCIn
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from custom_interfaces.action import Takeoff as Takeoff

class Takeoff(py_trees.behaviour.Behaviour):

    def __init__(self) -> None:
        super().__init__(name="Takeoff")

        self.blackboard = self.attach_blackboard_client(name="Takeoff")
        self.blackboard.register_key(key="altitude", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="longitude", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="latitude", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        """
        Called once during tree.setup().
        """

        self._node = kwargs["node"]
        self._rc_subscriber=self._node.create_subscription(RCIn, "/rc", self.rc_callback, 10)
        self._takeoff_action_client=ActionClient(self._node, Takeoff, "/takeoff")
    
    def rc_callback(self, msg: RCIn):
        self.channel6=msg.channels[6]

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self._goal_handle=None
        self.channel6=988
        self.cancelling=False
        self.status=None
        while not self._takeoff_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().info("Waiting for takeoff action server")
        goal=Takeoff.Goal()
        self.goal_future=self._takeoff_action_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self.goal_response_callback)
   
    def goal_response_callback(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected')
            self.status=GoalStatus.STATUS_ABORTED
            return
        self._node.get_logger().info('Goal accepted')
        self._goal_handle=goal_handle
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        response=future.result()
        result=response.result
        self.blackboard.altitude=result.altitude
        self.blackboard.longitude=result.longitude
        self.blackboard.latitude=result.latitude
        if response.status==GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info("Takeoff Action Succeeded")
        elif response.status==GoalStatus.STATUS_CANCELED:
            self._node.get_logger().info("Takeoff Action Ended Early")
        self.status=response.status


    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        if self.channel6 is not None and self.channel6>1400:
            if self.cancelling:
                pass
            else:
                self.cancelling=True
                self._node.get_logger().info("Takeoff Action Cancelled")
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
        if self.status is not None and self.status!=GoalStatus.STATUS_ABORTED:
            return py_trees.common.Status.SUCCESS
        elif self.status==GoalStatus.STATUS_ABORTED:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
