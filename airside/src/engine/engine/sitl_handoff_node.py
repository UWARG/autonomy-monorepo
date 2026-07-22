"""SITL-only GUIDED/arm/takeoff automation; never launched in production."""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from handoff import HandoffAction, HandoffSequencer
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from stack_config import DEPLOYED


class SitlHandoffNode(Node):
    def __init__(self) -> None:
        super().__init__("sitl_handoff")
        self._state: Optional[State] = None
        self._altitude = 0.0
        self._sequencer = HandoffSequencer(DEPLOYED.handoff)
        self._mode = self.create_client(SetMode, "mavros/set_mode")
        self._arm = self.create_client(CommandBool, "mavros/cmd/arming")
        self._takeoff = self.create_client(CommandTOL, "mavros/cmd/takeoff")
        self.create_subscription(State, "mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "mavros/local_position/pose", self._on_pose, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().warn("SITL handoff enabled: this node may change mode, arm, and take off")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, state: State) -> None:
        self._state = state

    def _on_pose(self, pose: PoseStamped) -> None:
        self._altitude = float(pose.pose.position.z)

    def _tick(self) -> None:
        state = self._state
        action = self._sequencer.step(
            connected=bool(state and state.connected),
            mode=state.mode if state else "",
            armed=bool(state and state.armed),
            alt_m=self._altitude,
            now_s=self._now(),
        )
        if action is HandoffAction.REQUEST_GUIDED and self._mode.service_is_ready():
            request = SetMode.Request()
            request.custom_mode = "GUIDED"
            self._mode.call_async(request)
        elif action is HandoffAction.REQUEST_ARM and self._arm.service_is_ready():
            request = CommandBool.Request()
            request.value = True
            self._arm.call_async(request)
        elif action is HandoffAction.REQUEST_TAKEOFF and self._takeoff.service_is_ready():
            request = CommandTOL.Request()
            request.altitude = float(DEPLOYED.handoff.takeoff_alt_m)
            self._takeoff.call_async(request)
            self._sequencer.notify_takeoff_sent(self._now())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SitlHandoffNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
