"""
Vision-map takeoff for the non-optimal/jetson tree.

Distinct from ``engine.behaviors.navigation.takeoff`` (MAVROS CommandTOL).
Writes launch latitude/longitude/altitude for ReturnToLaunch.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from mavros_msgs.msg import RCIn
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from custom_interfaces.action import Takeoff as TakeoffAction

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
        self._takeoff_action_client=ActionClient(self._node, TakeoffAction, "/takeoff")
    
    def rc_callback(self, msg: RCIn):
        self.channel7=msg.channels[6]

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self._goal_handle=None
        self.channel7=988
        self.cancelling=False
        self.success=None
        self._ended_early=False
        while not self._takeoff_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().info("Waiting for takeoff action server")
        goal=TakeoffAction.Goal()
        self.goal_future=self._takeoff_action_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self.goal_response_callback)
   
    def goal_response_callback(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected')
            self.success=GoalStatus.STATUS_ABORTED
            return
        self._node.get_logger().info('Goal accepted')
        self._goal_handle=goal_handle
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        response=future.result()
        self._node.get_logger().info("Get result callback "+str(response.status))
        result=response.result
        self.blackboard.altitude=result.altitude
        self.blackboard.longitude=result.longitude
        self.blackboard.latitude=result.latitude
        self._node.get_logger().info("altitude "+str(result.altitude)+" longitude "+str(result.longitude)+" latitude "+str(result.latitude))
        if response.status==GoalStatus.STATUS_UNKNOWN:
            self._node.get_logger().warning("Takeoff returned UNKNOWN; retrying")
            self._goal_handle=None
            goal=TakeoffAction.Goal()
            self.goal_future=self._takeoff_action_client.send_goal_async(goal)
            self.goal_future.add_done_callback(self.goal_response_callback)
            return
        if response.status==GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info("Takeoff Action Succeeded")
        elif response.status==GoalStatus.STATUS_CANCELED:
            self._node.get_logger().info("Takeoff Action Ended Early")
            self._ended_early=True
        self.success=response.status


    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        if self.channel7 is not None and self.channel7>1400:
            if self.cancelling:
                pass
            else:
                self.cancelling=True
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
        if self.success is not None and self.success!=GoalStatus.STATUS_ABORTED:
            return py_trees.common.Status.SUCCESS
        elif self.success==GoalStatus.STATUS_ABORTED:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            return
        if self._ended_early:
            self._node.get_logger().info("Mode switch: early takeoff -> flight")
        else:
            self._node.get_logger().info("Mode switch: takeoff -> flight "+str(new_status))
