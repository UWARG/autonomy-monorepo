from __future__ import annotations

import math
import threading

from utils.src.types import Attitude, PositionMessage


def _quaternion_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """(roll, pitch, yaw) radians from a (w, x, y, z) quaternion, ZYX aerospace convention."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class Telemetry:
    """Subscribes to MAVROS topics in the background and caches the latest values."""

    def __init__(self, node_name: str = "ims_telemetry") -> None:
        self._node_name = node_name
        self._node = None
        self._spin_thread: threading.Thread | None = None

        self._state = None
        self._pose = None
        self._fix = None
        self._last_statustext: str | None = None

    def connect(self) -> bool:
        """Start spinning a MAVROS-subscribed node in the background."""
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State, StatusText
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import NavSatFix

        if not rclpy.ok():
            rclpy.init()

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        self._node = Node(self._node_name)
        self._node.create_subscription(State, "mavros/state", self._on_state, state_qos)
        self._node.create_subscription(
            PoseStamped, "mavros/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self._node.create_subscription(
            NavSatFix, "mavros/global_position/global", self._on_fix, qos_profile_sensor_data
        )
        self._node.create_subscription(
            StatusText, "mavros/statustext/recv", self._on_statustext, qos_profile_sensor_data
        )

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()
        return True

    def is_connected(self) -> bool:
        return self._state is not None and self._state.connected

    def receive_attitude(self, attitude: Attitude) -> bool:
        """Fill `attitude` from the latest local_position/pose orientation."""
        if self._pose is None:
            return False

        q = self._pose.pose.orientation
        attitude.roll, attitude.pitch, attitude.yaw = _quaternion_to_euler(q.w, q.x, q.y, q.z)
        attitude.rollspeed = 0.0
        attitude.pitchspeed = 0.0
        attitude.yawspeed = 0.0
        return True

    def receive_position(self, position: PositionMessage) -> bool:
        """Fill `position` from the latest global_position/global fix."""
        if self._fix is None:
            return False

        position.lat = self._fix.latitude
        position.lon = self._fix.longitude
        position.alt = self._fix.altitude
        return True

    def receive_message(self) -> str | None:
        """Return the most recent STATUSTEXT received from the FCU, or None."""
        return self._last_statustext

    def _on_state(self, msg) -> None:
        self._state = msg

    def _on_pose(self, msg) -> None:
        self._pose = msg

    def _on_fix(self, msg) -> None:
        self._fix = msg

    def _on_statustext(self, msg) -> None:
        self._last_statustext = msg.text
