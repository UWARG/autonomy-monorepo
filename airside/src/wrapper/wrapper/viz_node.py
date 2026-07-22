from __future__ import annotations

import math
from typing import Optional

import rclpy
import rclpy.node
from airside_interfaces.msg import TrackedTarget
from rclpy.qos import qos_profile_sensor_data

from frames import body_to_ned, camera_to_body, ned_to_enu
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from stack_config import DEPLOYED
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

FRAME = "map"
PUBLISH_HZ = 10.0
STALE_S = 1.0

def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c

def _yaw_enu(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class VizNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("viz_node")
        self._pose: Optional[PoseStamped] = None
        self._pose_rx = 0.0
        self._target: Optional[TrackedTarget] = None
        self._target_rx = 0.0
        self._state: Optional[State] = None
        self._vel_sp: Optional[PositionTarget] = None
        self._vel_sp_rx = 0.0
        self._hold_sp: Optional[PoseStamped] = None
        self._hold_sp_rx = 0.0

        self.create_subscription(
            PoseStamped, "mavros/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(TrackedTarget, "perception/target", self._on_target, 10)
        self.create_subscription(State, "mavros/state", self._on_state, 10)
        self.create_subscription(
            PositionTarget, "mavros/setpoint_raw/local", self._on_vel_sp, 10
        )
        self.create_subscription(
            PoseStamped, "mavros/setpoint_position/local", self._on_hold_sp, 10
        )

        self._markers_pub = self.create_publisher(MarkerArray, "viz/markers", 10)
        self._range_pub = self.create_publisher(Float32, "viz/range", 10)
        self._state_pub = self.create_publisher(String, "viz/state", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info("viz_node: publishing viz/markers + viz/range + viz/state")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose, self._pose_rx = msg, self._now()

    def _on_target(self, msg: TrackedTarget) -> None:
        self._target, self._target_rx = msg, self._now()

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_vel_sp(self, msg: PositionTarget) -> None:
        self._vel_sp, self._vel_sp_rx = msg, self._now()

    def _on_hold_sp(self, msg: PoseStamped) -> None:
        self._hold_sp, self._hold_sp_rx = msg, self._now()

    # --- marker builders ---
    def _marker(self, mid: int, mtype: int, ns: str = "follow") -> Marker:
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.5e9)  # auto-expire if we stop publishing
        return m

    def _ring(self, mid: int, cx: float, cy: float, cz: float, radius: float,
              color: ColorRGBA) -> Marker:
        m = self._marker(mid, Marker.LINE_STRIP)
        m.scale.x = 0.04
        m.color = color
        n = 48
        for k in range(n + 1):
            a = 2.0 * math.pi * k / n
            p = PointStamped().point
            p.x, p.y, p.z = cx + radius * math.cos(a), cy + radius * math.sin(a), cz
            m.points.append(p)
        return m

    def _person_enu(self):
        """Camera-frame target -> world ENU position, or None."""
        if self._pose is None or self._target is None:
            return None
        if (self._now() - self._target_rx) > STALE_S:
            return None
        p = self._target.position
        body = camera_to_body(
            p.x * 1000.0, p.y * 1000.0, p.z * 1000.0,
            mount_pitch_rad=DEPLOYED.follow.mount_pitch_rad,
            mount_roll_rad=DEPLOYED.follow.mount_roll_rad,
        )
        yaw_ned = math.pi / 2.0 - _yaw_enu(self._pose)  # ENU CCW yaw -> NED CW yaw
        offset_enu = ned_to_enu(body_to_ned(body, yaw_ned))
        d = self._pose.pose.position
        return (d.x + offset_enu.x, d.y + offset_enu.y, d.z + offset_enu.z)

    def _tick(self) -> None:
        now = self._now()
        arr = MarkerArray()

        if self._pose is not None and (now - self._pose_rx) <= STALE_S:
            d = self._pose.pose.position

            drone = self._marker(1, Marker.SPHERE)
            drone.pose = self._pose.pose
            drone.scale.x = drone.scale.y = 0.5
            drone.scale.z = 0.2
            drone.color = _color(0.2, 0.4, 1.0)
            arr.markers.append(drone)

            heading = self._marker(2, Marker.ARROW)
            heading.pose = self._pose.pose  # arrow along body +x (FLU forward)
            heading.scale.x, heading.scale.y, heading.scale.z = 0.8, 0.06, 0.06
            heading.color = _color(0.2, 0.4, 1.0, 0.8)
            arr.markers.append(heading)

            # commanded velocity arrow (body-FLU velocity rotated by the pose)
            if self._vel_sp is not None and (now - self._vel_sp_rx) <= STALE_S:
                v = self._vel_sp.velocity
                yaw = _yaw_enu(self._pose)
                wx = v.x * math.cos(yaw) - v.y * math.sin(yaw)
                wy = v.x * math.sin(yaw) + v.y * math.cos(yaw)
                arrow = self._marker(3, Marker.ARROW)
                start = PointStamped().point
                start.x, start.y, start.z = d.x, d.y, d.z
                end = PointStamped().point
                end.x, end.y, end.z = d.x + 2.0 * wx, d.y + 2.0 * wy, d.z + 2.0 * v.z
                arrow.points = [start, end]
                arrow.scale.x, arrow.scale.y = 0.05, 0.12
                arrow.color = _color(0.0, 0.9, 0.9)
                arr.markers.append(arrow)

            person = self._person_enu()
            if person is not None:
                px, py, pz = person
                ball = self._marker(4, Marker.SPHERE)
                ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = px, py, pz
                ball.pose.orientation.w = 1.0
                ball.scale.x = ball.scale.y = 0.4
                ball.scale.z = 1.6  # person-ish
                ball.color = _color(0.1, 0.8, 0.1)
                arr.markers.append(ball)
                arr.markers.append(
                    self._ring(5, px, py, pz, DEPLOYED.follow.standoff_m, _color(1.0, 0.6, 0.0))
                )
                arr.markers.append(
                    self._ring(6, px, py, pz, DEPLOYED.follow.hard_min_m, _color(1.0, 0.1, 0.1))
                )
                rng = Float32()
                rng.data = float(
                    math.dist((d.x, d.y, d.z), (px, py, pz))
                )
                self._range_pub.publish(rng)

        # latched hold pose (recent republish on the position channel = hold active)
        if self._hold_sp is not None and (now - self._hold_sp_rx) <= STALE_S:
            h = self._marker(7, Marker.CUBE)
            h.pose = self._hold_sp.pose
            h.scale.x = h.scale.y = h.scale.z = 0.25
            h.color = _color(0.9, 0.2, 0.9)
            arr.markers.append(h)

        if arr.markers:
            self._markers_pub.publish(arr)

        status = String()
        mode = self._state.mode if self._state else "?"
        armed = bool(self._state and self._state.armed)
        target_age = (now - self._target_rx) if self._target else math.inf
        holding = self._hold_sp is not None and (now - self._hold_sp_rx) <= STALE_S
        status.data = (
            f"mode={mode} armed={armed} holding={holding} "
            f"target_age={min(target_age, 99.0):.2f}s"
        )
        self._state_pub.publish(status)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
