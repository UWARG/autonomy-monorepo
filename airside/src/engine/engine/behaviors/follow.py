"""Follow behavior: turn the cached target into a body-frame velocity command"""
from __future__ import annotations

import py_trees
import rclpy.node
from follow_controller import FollowConfig, compute_setpoint

from engine.behaviors.read_target import (
    KEY_COMMAND,
    KEY_COMMAND_STAMP,
    KEY_TARGET_M,
)

MM_PER_M = 1000.0

class FollowBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "Follow", config: FollowConfig | None = None) -> None:
        super().__init__(name=name)
        self._config = config or FollowConfig()
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=KEY_TARGET_M, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=KEY_COMMAND, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=KEY_COMMAND_STAMP, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def _write_command(self, cmd) -> None:
        self.blackboard.set(KEY_COMMAND, cmd)
        self.blackboard.set(KEY_COMMAND_STAMP, self._node.get_clock().now().nanoseconds * 1e-9)

    def update(self) -> py_trees.common.Status:
        target = self.blackboard.get(KEY_TARGET_M)
        if target is None:
            self._write_command((0.0, 0.0, 0.0, 0.0))
            return py_trees.common.Status.RUNNING

        x_m, y_m, z_m = target
        sp = compute_setpoint(x_m * MM_PER_M, y_m * MM_PER_M, z_m * MM_PER_M, self._config)
        self._write_command((sp.v_forward, sp.v_right, sp.v_down, sp.yaw_rate))
        self._node.get_logger().info(
            f"follow: range={sp.range_3d_m:.2f} horiz={sp.horiz_range_m:.2f} "
            f"bearing={sp.bearing_rad:.2f} vfwd={sp.v_forward:.2f} "
            f"vdown={sp.v_down:.2f} yaw_rate={sp.yaw_rate:.2f}"
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass
