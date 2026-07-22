"""EmergencyStop behavior"""

from __future__ import annotations

import py_trees
import rclpy.node
from safety import EStopAction, SafetyConfig, SafetyMonitor

from engine.behaviors.read_target import (
    KEY_COMMAND,
    KEY_COMMAND_STAMP,
    KEY_ESTOP_EMERGENCY,
    KEY_ESTOP_HOLD,
    KEY_ESTOP_RECEDE,
    KEY_RANGE_M,
)


class EmergencyStopBehavior(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str = "EmergencyStop",
        config: SafetyConfig | None = None,
        recede_speed: float = 1.0,
    ) -> None:
        super().__init__(name=name)
        self._monitor = SafetyMonitor(config or SafetyConfig())
        self._recede_speed = recede_speed
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=KEY_RANGE_M, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=KEY_COMMAND, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=KEY_COMMAND_STAMP, access=py_trees.common.Access.WRITE)
        for key in (KEY_ESTOP_EMERGENCY, KEY_ESTOP_RECEDE, KEY_ESTOP_HOLD):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._last_s = self._now_s()

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _write_command(self, cmd) -> None:
        self.blackboard.set(KEY_COMMAND, cmd)
        self.blackboard.set(KEY_COMMAND_STAMP, self._now_s())

    def update(self) -> py_trees.common.Status:
        now = self._now_s()
        dt = max(now - self._last_s, 1e-3)
        self._last_s = now

        rng = self.blackboard.get(KEY_RANGE_M)
        verdict = self._monitor.evaluate(rng, dt)

        self.blackboard.set(KEY_ESTOP_EMERGENCY, verdict.is_emergency)
        self.blackboard.set(KEY_ESTOP_RECEDE, verdict.recede)
        self.blackboard.set(
            KEY_ESTOP_HOLD,
            verdict.action is EStopAction.ZERO_VELOCITY and not verdict.is_emergency,
        )

        if verdict.action is EStopAction.NONE:
            return py_trees.common.Status.FAILURE  # yield to Follow

        if verdict.recede:
            self._write_command((-self._recede_speed, 0.0, 0.0, 0.0))
        else:
            self._write_command((0.0, 0.0, 0.0, 0.0))

        if verdict.is_emergency:
            self._node.get_logger().warn(f"EmergencyStop ENGAGED: {verdict.reason}")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass
