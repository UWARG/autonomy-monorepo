"""
Single point of contact between the engine and MAVROS position setpoints.

Behaviors publish where they want the drone to go on ``position_controller/target``;
this node turns each target into a MAVROS guided-mode global setpoint. Keeping
this hop in its own node leaves room to adjust targets (e.g. greedy obstacle
avoidance) without touching the behaviors.
"""

from __future__ import annotations

import rclpy
import rclpy.node
from airside_interfaces.msg import Coordinate
from mavros_msgs.msg import GlobalPositionTarget


class PositionController(rclpy.node.Node):
    """
    Forwards every ``Coordinate`` target (lat, lon, relative alt) to
    ``mavros/setpoint_raw/global`` as a position-only setpoint.
    """

    TARGET_TOPIC = "position_controller/target"
    SETPOINT_TOPIC = "mavros/setpoint_raw/global"

    # Position-only setpoint: ignores velocity, acceleration and yaw fields
    TYPE_MASK = (
        GlobalPositionTarget.IGNORE_VX
        | GlobalPositionTarget.IGNORE_VY
        | GlobalPositionTarget.IGNORE_VZ
        | GlobalPositionTarget.IGNORE_AFX
        | GlobalPositionTarget.IGNORE_AFY
        | GlobalPositionTarget.IGNORE_AFZ
        | GlobalPositionTarget.IGNORE_YAW
        | GlobalPositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self) -> None:
        super().__init__("position_controller")

        self._setpoint_pub = self.create_publisher(
            msg_type=GlobalPositionTarget,
            topic=self.SETPOINT_TOPIC,
            qos_profile=10,
        )
        self._target_sub = self.create_subscription(
            msg_type=Coordinate,
            topic=self.TARGET_TOPIC,
            callback=self._target_callback,
            qos_profile=10,
        )

        self.get_logger().info(
            f"Position controller ready - forwarding '{self.TARGET_TOPIC}' "
            f"to '{self.SETPOINT_TOPIC}'."
        )

    def _target_callback(self, target: Coordinate) -> None:
        setpoint = GlobalPositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        setpoint.type_mask = self.TYPE_MASK
        setpoint.latitude = target.lat
        setpoint.longitude = target.lon
        setpoint.altitude = target.alt
        self._setpoint_pub.publish(setpoint)

        self.get_logger().debug(
            f"Setpoint lat={target.lat:.7f} lon={target.lon:.7f} alt={target.alt:.1f}m"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
