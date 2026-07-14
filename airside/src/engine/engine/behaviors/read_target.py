"""ReadTarget behavior: cache the latest camera-relative human position"""

from __future__ import annotations

import math

import py_trees
import rclpy.node
from geometry_msgs.msg import PointStamped
from stack_config import DEPLOYED

KEY_TARGET_M = "follow/target_m"  # (x, y, z) camera-frame metres, or None
KEY_RANGE_M = "follow/range_m"  # 3D range in metres, or None
KEY_COMMAND = "follow/command"  # (v_forward, v_right, v_down, yaw_rate) FRD, or None
KEY_COMMAND_STAMP = "follow/command_stamp"  # node-clock seconds the command was written
KEY_ESTOP_EMERGENCY = "follow/estop_emergency"  # bool: a latched emergency
KEY_ESTOP_RECEDE = "follow/estop_recede"  # bool: proximity danger (True) vs target-lost (False)
KEY_ESTOP_HOLD = "follow/estop_hold"  # bool: a non-emergency hold (e.g. brief dropout)


class ReadTargetBehavior(py_trees.behaviour.Behaviour):
    TOPIC = "perception/target"
    FRESHNESS_S = DEPLOYED.target_freshness_s  # single source of truth

    def __init__(self, name: str = "ReadTarget") -> None:
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=KEY_TARGET_M, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=KEY_RANGE_M, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest: PointStamped | None = None
        self._latest_rx_s: float = 0.0
        self._sub = self._node.create_subscription(
            PointStamped, self.TOPIC, self._callback, qos_profile=10
        )
        self._node.get_logger().info(f"ReadTarget: subscribed to '{self.TOPIC}'")

    def _callback(self, msg: PointStamped) -> None:
        self._latest = msg
        self._latest_rx_s = self._now_s()

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def update(self) -> py_trees.common.Status:
        target = None
        rng = None
        if self._latest is not None and (self._now_s() - self._latest_rx_s) <= self.FRESHNESS_S:
            p = self._latest.point
            if math.isfinite(p.z) and p.z > 0.0 and math.isfinite(p.x) and math.isfinite(p.y):
                target = (p.x, p.y, p.z)
                rng = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        self.blackboard.set(KEY_TARGET_M, target)
        self.blackboard.set(KEY_RANGE_M, rng)
        return py_trees.common.Status.SUCCESS
