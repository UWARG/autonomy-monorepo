from __future__ import annotations

import sys
import time

import rclpy
from airside_interfaces.msg import TrackedTarget
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from std_srvs.srv import SetBool

CASES = [
    ("ahead -> velocity.x > 0 (forward)", (0.0, 0.0, 6.0), lambda m: m.velocity.x > 0.05),
    ("too close -> velocity.x < 0 (back away)", (0.0, 0.0, 2.0), lambda m: m.velocity.x < -0.01),
    ("right -> yaw_rate < 0 (CW turn, CCW-positive frame)", (1.0, 0.0, 3.0),
     lambda m: m.yaw_rate < -0.01),
    ("left -> yaw_rate > 0 (CCW turn)", (-1.0, 0.0, 3.0), lambda m: m.yaw_rate > 0.01),
    ("below -> velocity.z < 0 (descend, FLU up-positive)", (0.0, 0.5, 3.0),
     lambda m: m.velocity.z < -0.01),
]

CASE_GAP_S = 0.15

class SignCheck(Node):
    def __init__(self) -> None:
        super().__init__("sign_check")
        self._pub = self.create_publisher(TrackedTarget, "perception/target", 10)
        self._candidate_pub = self.create_publisher(
            TrackedTarget, "perception/target_candidate", 10
        )
        self._enable = self.create_client(SetBool, "follow/set_enabled")
        self._sequence = 0
        self._msg: PositionTarget | None = None
        self.create_subscription(PositionTarget, "mavros/setpoint_raw/local", self._on_sp, 10)

    def _on_sp(self, msg: PositionTarget) -> None:
        self._msg = msg

    def _publish(self, point) -> None:
        self._sequence += 1
        msg = TrackedTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.host_receipt_stamp = msg.header.stamp
        msg.publish_stamp = msg.header.stamp
        msg.header.frame_id = "camera"
        msg.position.x, msg.position.y, msg.position.z = point
        msg.track_id = 1
        msg.sequence_num = self._sequence
        self._candidate_pub.publish(msg)
        self._pub.publish(msg)

    def _request_enable(self) -> None:
        if not self._enable.wait_for_service(timeout_sec=2.0):
            return
        request = SetBool.Request()
        request.data = True
        self._enable.call_async(request)

    def _silent_gap(self) -> None:
        deadline = time.time() + CASE_GAP_S
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> bool:
        all_ok = True
        for label, point, predicate in CASES:
            self._silent_gap()
            self._publish(point)
            rclpy.spin_once(self, timeout_sec=0.1)
            self._request_enable()
            deadline = time.time() + 3.5
            self._msg = None
            settle = time.time() + 2.0
            while time.time() < deadline:
                self._publish(point)
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.time() < settle:
                    self._msg = None  # discard transitional setpoints
            ok = self._msg is not None and predicate(self._msg)
            all_ok = all_ok and ok
            detail = ""
            if self._msg is not None:
                detail = (
                    f"  (vx={self._msg.velocity.x:+.2f} vy={self._msg.velocity.y:+.2f} "
                    f"vz={self._msg.velocity.z:+.2f} yaw_rate={self._msg.yaw_rate:+.2f})"
                )
            else:
                detail = "  (no setpoint received -- not in GUIDED?)"
            self.get_logger().info(f"[{'PASS' if ok else 'FAIL'}] {label}{detail}")
        return all_ok


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SignCheck()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("SIGN-AND-MASK GATE:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
