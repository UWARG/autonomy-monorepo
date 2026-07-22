"""Five-minute props-off HITL event recorder for follow-stack Gate 5."""

from __future__ import annotations

import argparse
import csv
import os
import sys

import rclpy
import rclpy.node
from airside_interfaces.msg import TrackedTarget
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import String

FIELDS = [
    "event",
    "host_time_s",
    "capture_time_s",
    "device_host_receipt_s",
    "ros_publish_s",
    "ros_receive_s",
    "latest_capture_s",
    "sequence_num",
    "sequence_gap",
    "track_id",
    "mode",
    "armed",
    "x",
    "y",
    "z",
    "vehicle_x",
    "vehicle_y",
    "vehicle_z",
    "vehicle_vx",
    "vehicle_vy",
    "vehicle_vz",
    "sp_vx",
    "sp_vy",
    "sp_vz",
    "sp_yaw_rate",
    "authority_state",
    "stop_reason",
    "scenario",
]


def stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Recorder(rclpy.node.Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("follow_hitl_recorder")
        parent = os.path.dirname(os.path.abspath(out_path))
        os.makedirs(parent, exist_ok=True)
        self._file = open(out_path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._rows = 0
        self._state = None
        self._last_mode = None
        self._latest_capture_s = None
        self._last_sequence = None
        self._scenario = ""
        self._pose = None
        self._velocity = None

        self.create_subscription(State, "mavros/state", self._on_state, 10)
        self.create_subscription(TrackedTarget, "perception/target", self._on_target, 10)
        self.create_subscription(PositionTarget, "mavros/setpoint_raw/local", self._on_sp, 10)
        self.create_subscription(DiagnosticArray, "follow/diagnostics", self._on_diag, 10)
        self.create_subscription(String, "follow/hitl_scenario", self._on_scenario, 10)
        self.create_subscription(PoseStamped, "mavros/local_position/pose", self._on_pose, 10)
        self.create_subscription(
            TwistStamped, "mavros/local_position/velocity_local", self._on_velocity, 10
        )
        self.get_logger().info(
            f"recording Gate-5 events to {out_path}; publish scenario labels on "
            "'/follow/hitl_scenario'"
        )
        self._write("session_start")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _write(self, event: str, **values) -> None:
        row = {field: "" for field in FIELDS}
        row.update(
            event=event,
            host_time_s=f"{self._now():.9f}",
            latest_capture_s=(
                f"{self._latest_capture_s:.9f}" if self._latest_capture_s else ""
            ),
            mode=self._state.mode if self._state else "",
            armed=int(bool(self._state and self._state.armed)),
            scenario=self._scenario,
            vehicle_x=(f"{self._pose.pose.position.x:.6f}" if self._pose else ""),
            vehicle_y=(f"{self._pose.pose.position.y:.6f}" if self._pose else ""),
            vehicle_z=(f"{self._pose.pose.position.z:.6f}" if self._pose else ""),
            vehicle_vx=(f"{self._velocity.twist.linear.x:.6f}" if self._velocity else ""),
            vehicle_vy=(f"{self._velocity.twist.linear.y:.6f}" if self._velocity else ""),
            vehicle_vz=(f"{self._velocity.twist.linear.z:.6f}" if self._velocity else ""),
        )
        row.update(values)
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % 100 == 0:
            self._file.flush()

    def _on_state(self, message: State) -> None:
        self._state = message
        if message.mode != self._last_mode:
            self._last_mode = message.mode
            self._write("mode_transition")

    def _on_scenario(self, message: String) -> None:
        self._scenario = message.data.strip()
        self._write("scenario")

    def _on_velocity(self, message: TwistStamped) -> None:
        self._velocity = message

    def _on_pose(self, message: PoseStamped) -> None:
        self._pose = message
        self._write("vehicle")

    def _on_target(self, message: TrackedTarget) -> None:
        sequence = int(message.sequence_num)
        gap = (
            max(0, sequence - self._last_sequence - 1)
            if self._last_sequence is not None and sequence > self._last_sequence
            else 0
        )
        self._last_sequence = sequence
        self._latest_capture_s = stamp_s(message.header.stamp)
        self._write(
            "target",
            capture_time_s=f"{self._latest_capture_s:.9f}",
            device_host_receipt_s=f"{stamp_s(message.host_receipt_stamp):.9f}",
            ros_publish_s=f"{stamp_s(message.publish_stamp):.9f}",
            ros_receive_s=f"{self._now():.9f}",
            sequence_num=sequence,
            sequence_gap=gap,
            track_id=int(message.track_id),
            x=f"{message.position.x:.6f}",
            y=f"{message.position.y:.6f}",
            z=f"{message.position.z:.6f}",
        )

    def _on_sp(self, message: PositionTarget) -> None:
        self._write(
            "setpoint",
            sp_vx=f"{message.velocity.x:.6f}",
            sp_vy=f"{message.velocity.y:.6f}",
            sp_vz=f"{message.velocity.z:.6f}",
            sp_yaw_rate=f"{message.yaw_rate:.6f}",
        )

    def _on_diag(self, message: DiagnosticArray) -> None:
        if not message.status:
            return
        values = {item.key: item.value for item in message.status[0].values}
        self._write(
            "diagnostic",
            authority_state=values.get("authority_state", ""),
            stop_reason=values.get("stop_reason", ""),
        )

    def close(self) -> None:
        self._write("session_end")
        self._file.flush()
        self._file.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, default=300.0)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.out)
    try:
        deadline = node._now() + args.duration
        while rclpy.ok() and node._now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
