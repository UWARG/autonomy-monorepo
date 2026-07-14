from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from camera.src.target_source import AbstractTargetSource, SimTargetSource

MM_PER_M = 1000.0
SETUP_DISTANCE_M = 8.0  # how far ahead the world-fixed person is placed at takeoff
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
        self._world = self.get_parameter("world_target").value
        self._lunge = self.get_parameter("lunge").value

        self._source: AbstractTargetSource = SimTargetSource()
        self._source.initialize()
        self._pose: Optional[PoseStamped] = None
        self._person: Optional[tuple] = None  # latched world-fixed (x, y, z) ENU
        self._latch_s: Optional[float] = None  # when the person was latched (lunge timing)

        self._publisher = self.create_publisher(PointStamped, self.TOPIC, 10)
        if self._world:
            # MAVROS publishes pose with best-effort (SensorData) QoS.
            self.create_subscription(
                PoseStamped, self.POSE_TOPIC, self._on_pose, qos_profile_sensor_data
            )
        self.create_timer(1.0 / self.PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f"SimTargetNode: publishing '{self.TOPIC}' at {self.PUBLISH_HZ} Hz "
            f"(world_target={self._world})."
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _yaw_from(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _world_camera_xyz(self):
        """Camera-frame (x=right, y=down, z=forward) metres of the world-fixed person."""
        if self._pose is None:
            return None
        p = self._pose.pose.position
        if p.z < 1.0:  # latch as soon as airborne (reliable), but at a FIXED altitude below --
            return None  # latching at the drone's mid-climb alt would pull it back down.
        yaw = self._yaw_from(self._pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        if self._person is None:
            # Latch the person SETUP_DISTANCE ahead of the drone, at the fixed person altitude
            # (the drone then climbs/holds to PERSON_ALT_M via vertical framing).
            self._person = (p.x + SETUP_DISTANCE_M * c, p.y + SETUP_DISTANCE_M * s, PERSON_ALT_M)
            self._latch_s = self._now_s()
            self.get_logger().warn(f"SimTargetNode: world person latched at {self._person}")

        # Gate-3c lunge: after the drone settles, the person charges the drone at a
        # fixed horizontal speed so we can watch the recede + closing-rate BRAKE reflex.
        if self._lunge and self._latch_s is not None and (self._now_s() - self._latch_s) > LUNGE_AFTER_S:
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
        if self._world:
            cam = self._world_camera_xyz()
            if cam is None:
                return
            x_m, y_m, z_m = cam
        else:
            obs = self._source.get_target()
            if obs is None or not obs.tracked or obs.z_mm <= 0.0:
                return
            x_m, y_m, z_m = obs.x_mm / MM_PER_M, obs.y_mm / MM_PER_M, obs.z_mm / MM_PER_M

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.point.x = x_m  # right
        msg.point.y = y_m  # down
        msg.point.z = z_m  # forward
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
