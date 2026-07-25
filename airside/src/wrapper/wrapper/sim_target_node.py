from __future__ import annotations

import json
import math
import random
from typing import Optional

import rclpy
from airside_interfaces.msg import TrackedTarget
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from camera.src.target_source import AbstractTargetSource, SimTargetSource

MM_PER_M = 1000.0
SETUP_DISTANCE_M = 3.0  # maximum validated acquisition range
PERSON_ALT_M = 3.0  # fixed altitude of the world-fixed person (== takeoff altitude)
MIN_FORWARD_M = 0.3  # below this the person is behind/at the camera -> not "seen"
LUNGE_SPEED_MPS = 0.8  # person walk-at-drone speed once the lunge starts
LUNGE_AFTER_S = 12.0  # let the drone climb, approach, and settle at standoff first
LUNGE_MIN_RANGE_M = 0.8  # don't script the person literally through the drone


class SimTargetNode(Node):
    TOPIC = "perception/target"
    POSE_TOPIC = "mavros/local_position/pose"
    PUBLISH_HZ = 20.0

    def __init__(self) -> None:
        super().__init__("sim_target_node")
        self.declare_parameter("world_target", False)
        self.declare_parameter("lunge", False)
        self.declare_parameter("crossing", False)
        self.declare_parameter("detector_stride", 1)
        self.declare_parameter("sim_latency_s", 0.0)
        self.declare_parameter("occlusion_after_s", -1.0)
        self.declare_parameter("occlusion_duration_s", 0.0)
        self.declare_parameter("drop_detector_every_n", 0)
        self.declare_parameter("timing_json", "")
        self._world = self.get_parameter("world_target").value
        self._lunge = self.get_parameter("lunge").value
        self._crossing = self.get_parameter("crossing").value
        self._detector_stride = int(self.get_parameter("detector_stride").value)
        if self._detector_stride not in (1, 2):
            raise ValueError("detector_stride must be 1 or 2")
        self._sim_latency_s = max(0.0, float(self.get_parameter("sim_latency_s").value))
        self._occlusion_after_s = float(self.get_parameter("occlusion_after_s").value)
        self._occlusion_duration_s = max(
            0.0, float(self.get_parameter("occlusion_duration_s").value)
        )
        self._drop_detector_every_n = max(
            0, int(self.get_parameter("drop_detector_every_n").value)
        )
        self._capture_jitter_s = 0.0
        self._latency_jitter_s = 0.0
        self._dropout_rate = 0.0
        self._dropout_gap_sizes = ()
        timing_json = str(self.get_parameter("timing_json").value)
        if timing_json:
            with open(timing_json, encoding="utf-8") as timing_file:
                timing = json.load(timing_file)
            detector_p05_fps = float(
                timing.get("detector_p05_fps", timing.get("p05_fps", 20.0))
            )
            if detector_p05_fps <= 0.0:
                raise ValueError("timing_json detector p05 FPS must be positive")
            self._detector_stride = max(
                1, min(2, round(self.PUBLISH_HZ / detector_p05_fps))
            )
            self._sim_latency_s = float(
                timing.get(
                    "detector_capture_to_ros_p99_s",
                    timing.get("capture_to_receive_p99_s", 0.0),
                )
            )
            self._capture_jitter_s = min(
                0.045,
                max(
                    0.0,
                    float(
                        timing.get(
                            "detector_capture_period_jitter_p99_s",
                            timing.get("capture_period_jitter_p99_s", 0.0),
                        )
                    ),
                ),
            )
            self._latency_jitter_s = max(
                0.0, float(timing.get("latency_jitter_p99_s", 0.0))
            )
            self._dropout_rate = max(
                0.0, min(1.0, float(timing.get("dropout_rate", 0.0)))
            )
            self._dropout_gap_sizes = tuple(
                max(1, int(value)) for value in timing.get("dropout_gap_sizes", [])
            )

        self._source: AbstractTargetSource = SimTargetSource(
            z_centre_mm=2600.0,
            z_amplitude_mm=200.0,
            x_amplitude_mm=300.0,
        )
        self._source.initialize()
        self._pose: Optional[PoseStamped] = None
        self._person: Optional[tuple] = None  # latched world-fixed (x, y, z) ENU
        self._latch_s: Optional[float] = (
            None  # when the person was latched (lunge timing)
        )
        self._sequence_num = 0
        self._detector_count = 0
        self._last_detector_stamp = None
        self._last_detector_sequence = 0
        self._started_s = self._now_s()
        self._pending = []
        self._forced_occlusion_until_s = 0.0
        self._rng = random.Random(42)
        self._dropout_remaining = 0

        self._publisher = self.create_publisher(TrackedTarget, self.TOPIC, 10)
        self._candidate_publisher = self.create_publisher(
            TrackedTarget, "perception/target_candidate", 10
        )
        if self._world:
            # MAVROS publishes pose with best-effort (SensorData) QoS.
            self.create_subscription(
                PoseStamped, self.POSE_TOPIC, self._on_pose, qos_profile_sensor_data
            )
        self.create_subscription(
            Float32,
            "follow/sim_occlusion_duration",
            self._on_occlusion,
            10,
        )
        self.create_subscription(Float32, "follow/sim_latency", self._on_latency, 10)
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f"SimTargetNode: publishing '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"(world_target={self._world}, detector_stride={self._detector_stride}, "
            f"latency={self._sim_latency_s:.3f}s, timing_json={timing_json or 'none'})."
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _on_occlusion(self, msg: Float32) -> None:
        duration_s = max(0.0, float(msg.data))
        self._forced_occlusion_until_s = self._now_s() + duration_s
        self.get_logger().warning(
            f"simulating total target occlusion for {duration_s:.3f}s"
        )

    def _on_latency(self, msg: Float32) -> None:
        self._sim_latency_s = max(0.0, float(msg.data))
        self.get_logger().warning(
            f"simulated capture-to-publish latency set to {self._sim_latency_s:.3f}s"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _yaw_from(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _world_camera_xyz(self):
        """Camera-frame (x=right, y=down, z=forward) metres of the world-fixed person."""
        if self._pose is None:
            return None
        p = self._pose.pose.position
        if (
            p.z < 1.0
        ):  # latch as soon as airborne (reliable), but at a FIXED altitude below --
            return (
                None  # latching at the drone's mid-climb alt would pull it back down.
            )
        yaw = self._yaw_from(self._pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        if self._person is None:
            # Latch the person SETUP_DISTANCE ahead of the drone, at the fixed person altitude
            # (the drone then climbs/holds to PERSON_ALT_M via vertical framing).
            self._person = (
                p.x + SETUP_DISTANCE_M * c,
                p.y + SETUP_DISTANCE_M * s,
                PERSON_ALT_M,
            )
            self._latch_s = self._now_s()
            self.get_logger().warn(
                f"SimTargetNode: world person latched at {self._person}"
            )

        # Gate-3c lunge: after the drone settles, the person charges the drone at a
        # fixed horizontal speed so we can watch the recede + closing-rate BRAKE reflex.
        if (
            self._lunge
            and self._latch_s is not None
            and (self._now_s() - self._latch_s) > LUNGE_AFTER_S
        ):
            px, py, pz = self._person
            dx, dy = p.x - px, p.y - py
            gap = math.hypot(dx, dy)
            if gap > LUNGE_MIN_RANGE_M:
                step = LUNGE_SPEED_MPS / self.PUBLISH_HZ
                self._person = (px + step * dx / gap, py + step * dy / gap, pz)

        rx, ry, rz = self._person[0] - p.x, self._person[1] - p.y, self._person[2] - p.z
        fwd = rx * c + ry * s
        left = -rx * s + ry * c
        if fwd <= MIN_FORWARD_M:
            return None  # behind / at the camera -> not visible this tick
        return (-left, -rz, fwd)  # (right, down, forward)

    def _publish(self) -> None:
        now_s = self._now_s()
        ready = [item for item in self._pending if item[0] <= now_s]
        self._pending = [item for item in self._pending if item[0] > now_s]
        for _, publisher, pending_message in ready:
            publisher.publish(pending_message)

        elapsed = now_s - self._started_s
        if now_s < self._forced_occlusion_until_s or (
            self._occlusion_after_s >= 0.0
            and self._occlusion_after_s
            <= elapsed
            < self._occlusion_after_s + self._occlusion_duration_s
        ):
            return
        if self._world:
            cam = self._world_camera_xyz()
            if cam is None:
                return
            x_m, y_m, z_m = cam
        else:
            obs = self._source.get_target()
            if obs is None or not obs.tracked or obs.z_mm <= 0.0:
                return
            x_m, y_m, z_m = (
                obs.x_mm / MM_PER_M,
                obs.y_mm / MM_PER_M,
                obs.z_mm / MM_PER_M,
            )

        self._sequence_num += 1
        msg = TrackedTarget()
        capture_offset_s = (
            self._capture_jitter_s if self._sequence_num % 2 == 0 else 0.0
        )
        msg.header.stamp = self._time_msg(now_s - capture_offset_s)
        msg.host_receipt_stamp = msg.header.stamp
        msg.publish_stamp = msg.header.stamp
        msg.header.frame_id = "camera"
        msg.position.x = x_m  # right
        msg.position.y = y_m  # down
        msg.position.z = z_m  # forward
        msg.track_id = 1
        msg.sequence_num = self._sequence_num
        detector_confirmed = (self._sequence_num - 1) % self._detector_stride == 0
        if detector_confirmed:
            self._detector_count += 1
            if self._dropout_remaining > 0:
                self._dropout_remaining -= 1
                detector_confirmed = False
            elif self._rng.random() < self._dropout_rate:
                detector_confirmed = False
                if self._dropout_gap_sizes:
                    self._dropout_remaining = (
                        self._rng.choice(self._dropout_gap_sizes) - 1
                    )
            if (
                self._drop_detector_every_n > 0
                and self._detector_count % self._drop_detector_every_n == 0
            ):
                detector_confirmed = False
        if detector_confirmed:
            self._last_detector_stamp = msg.header.stamp
            self._last_detector_sequence = msg.sequence_num
        msg.detector_stamp = self._last_detector_stamp or msg.header.stamp
        msg.detector_sequence_num = self._last_detector_sequence
        msg.detector_confirmed = detector_confirmed
        msg.within_validated_range = math.sqrt(x_m**2 + y_m**2 + z_m**2) <= 3.0
        self._publish_with_latency(self._publisher, msg, now_s)
        if self._crossing:
            bystander = TrackedTarget()
            bystander.header = msg.header
            bystander.host_receipt_stamp = msg.host_receipt_stamp
            bystander.publish_stamp = msg.publish_stamp
            bystander.position.x = -msg.position.x
            bystander.position.y = msg.position.y
            bystander.position.z = max(0.4, msg.position.z * 0.5)
            bystander.track_id = 99
            bystander.sequence_num = msg.sequence_num
            bystander.detector_stamp = msg.detector_stamp
            bystander.detector_sequence_num = msg.detector_sequence_num
            bystander.detector_confirmed = msg.detector_confirmed
            bystander.within_validated_range = True
            self._publish_with_latency(self._candidate_publisher, bystander, now_s)

    def _publish_with_latency(self, publisher, message, now_s: float) -> None:
        jitter = (
            self._latency_jitter_s
            if int(message.sequence_num) % 2 == 0
            else -self._latency_jitter_s
        )
        latency_s = max(0.0, self._sim_latency_s + jitter)
        if latency_s <= 0.0:
            publisher.publish(message)
            return
        self._pending.append((now_s + latency_s, publisher, message))

    @staticmethod
    def _time_msg(value_s: float):
        from builtin_interfaces.msg import Time

        stamp = Time()
        stamp.sec = math.floor(value_s)
        stamp.nanosec = int((value_s - stamp.sec) * 1e9)
        return stamp


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
