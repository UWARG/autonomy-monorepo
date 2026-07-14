import argparse
import csv
import os
import sys

import rclpy
import rclpy.node
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.qos import qos_profile_sensor_data

FIELDS = [
    "t", "x", "y", "z", "mode", "armed",
    "target_x", "target_y", "target_z", "target_age_s",
    "sp_vx", "sp_vy", "sp_vz", "sp_yaw_rate", "sp_age_s",
    "hold_x", "hold_y", "hold_z", "hold_age_s",
]


class Recorder(rclpy.node.Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("flight_recorder")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        self._file = open(out_path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._rows = 0

        self._state = None
        self._target = None
        self._target_rx = None
        self._sp = None
        self._sp_rx = None
        self._hold = None
        self._hold_rx = None

        self.create_subscription(
            PoseStamped, "mavros/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(State, "mavros/state", self._on_state, 10)
        self.create_subscription(PointStamped, "perception/target", self._on_target, 10)
        self.create_subscription(PositionTarget, "mavros/setpoint_raw/local", self._on_sp, 10)
        self.create_subscription(
            PoseStamped, "mavros/setpoint_position/local", self._on_hold, 10
        )
        self.get_logger().info(f"recording to {out_path}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg) -> None:
        self._state = msg

    def _on_target(self, msg) -> None:
        self._target, self._target_rx = msg, self._now()

    def _on_sp(self, msg) -> None:
        self._sp, self._sp_rx = msg, self._now()

    def _on_hold(self, msg) -> None:
        self._hold, self._hold_rx = msg, self._now()

    def _on_pose(self, msg) -> None:
        now = self._now()
        row = {
            "t": f"{now:.3f}",
            "x": f"{msg.pose.position.x:.3f}",
            "y": f"{msg.pose.position.y:.3f}",
            "z": f"{msg.pose.position.z:.3f}",
            "mode": self._state.mode if self._state else "",
            "armed": int(bool(self._state and self._state.armed)),
            "target_x": f"{self._target.point.x:.3f}" if self._target else "",
            "target_y": f"{self._target.point.y:.3f}" if self._target else "",
            "target_z": f"{self._target.point.z:.3f}" if self._target else "",
            "target_age_s": (
                f"{now - self._target_rx:.3f}" if self._target_rx is not None else ""
            ),
            "sp_vx": f"{self._sp.velocity.x:.3f}" if self._sp else "",
            "sp_vy": f"{self._sp.velocity.y:.3f}" if self._sp else "",
            "sp_vz": f"{self._sp.velocity.z:.3f}" if self._sp else "",
            "sp_yaw_rate": f"{self._sp.yaw_rate:.3f}" if self._sp else "",
            "sp_age_s": f"{now - self._sp_rx:.3f}" if self._sp_rx is not None else "",
            "hold_x": f"{self._hold.pose.position.x:.3f}" if self._hold else "",
            "hold_y": f"{self._hold.pose.position.y:.3f}" if self._hold else "",
            "hold_z": f"{self._hold.pose.position.z:.3f}" if self._hold else "",
            "hold_age_s": f"{now - self._hold_rx:.3f}" if self._hold_rx is not None else "",
        }
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % 100 == 0:
            self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = until killed")
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args.out)
    try:
        if args.duration > 0:
            end = node._now() + args.duration
            while rclpy.ok() and node._now() < end:
                rclpy.spin_once(node, timeout_sec=0.5)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
