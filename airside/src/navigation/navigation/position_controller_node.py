"""Single owner for Airside position and velocity setpoints sent to MAVROS."""

from __future__ import annotations

import math
import time

import rclpy
import rclpy.node
from airside_interfaces.msg import Coordinate
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GlobalPositionTarget, State

from navigation.setpoint_router import RoutingDecision, SetpointRouter


class PositionController(rclpy.node.Node):
    """Arbitrate Airside targets and exclusively publish MAVROS setpoints."""

    TARGET_TOPIC = "position_controller/target"
    VELOCITY_TARGET_TOPIC = "position_controller/velocity_target"
    STATE_TOPIC = "mavros/state"
    POSITION_SETPOINT_TOPIC = "mavros/setpoint_raw/global"
    VELOCITY_SETPOINT_TOPIC = "mavros/setpoint_velocity/cmd_vel"
    GUIDED_MODE = "GUIDED"
    DEFAULT_VELOCITY_LEASE_S = 0.3

    # Position-only setpoint: ignores velocity, acceleration and yaw fields.
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
        self.declare_parameter("velocity_lease_s", self.DEFAULT_VELOCITY_LEASE_S)
        velocity_lease_s = float(self.get_parameter("velocity_lease_s").value)
        self._router = SetpointRouter(velocity_lease_s)
        self._latest_state: State | None = None

        self._position_pub = self.create_publisher(
            GlobalPositionTarget,
            self.POSITION_SETPOINT_TOPIC,
            10,
        )
        self._velocity_pub = self.create_publisher(
            TwistStamped,
            self.VELOCITY_SETPOINT_TOPIC,
            10,
        )
        self._target_sub = self.create_subscription(
            Coordinate,
            self.TARGET_TOPIC,
            self._target_callback,
            10,
        )
        self._velocity_target_sub = self.create_subscription(
            TwistStamped,
            self.VELOCITY_TARGET_TOPIC,
            self._velocity_target_callback,
            10,
        )
        self._state_sub = self.create_subscription(
            State,
            self.STATE_TOPIC,
            self._state_callback,
            10,
        )
        self._watchdog_timer = self.create_timer(
            min(0.1, velocity_lease_s / 2.0),
            self._watchdog_cycle,
        )

        self.get_logger().info(
            "Position controller ready - sole MAVROS position/velocity "
            f"setpoint owner (velocity lease {velocity_lease_s:.3f}s)."
        )

    def _command_allowed(self) -> bool:
        return bool(
            self._latest_state is not None
            and self._latest_state.connected
            and self._latest_state.armed
            and self._latest_state.mode == self.GUIDED_MODE
        )

    def _target_callback(self, target: Coordinate) -> None:
        decision = self._router.receive_position(self._command_allowed())
        self._apply_routing(decision)
        if not decision.publish_position:
            return

        setpoint = GlobalPositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        setpoint.type_mask = self.TYPE_MASK
        setpoint.latitude = target.lat
        setpoint.longitude = target.lon
        setpoint.altitude = target.alt
        self._position_pub.publish(setpoint)
        self.get_logger().debug(
            f"Position setpoint lat={target.lat:.7f} lon={target.lon:.7f} "
            f"alt={target.alt:.1f}m"
        )

    def _velocity_target_callback(self, target: TwistStamped) -> None:
        components = (
            target.twist.linear.x,
            target.twist.linear.y,
            target.twist.linear.z,
        )
        command_valid = target.header.frame_id == "map" and all(
            math.isfinite(component) for component in components
        )
        decision = self._router.receive_velocity(
            now_s=time.monotonic(),
            command_allowed=self._command_allowed(),
            command_valid=command_valid,
        )
        self._apply_routing(decision)
        if not decision.publish_velocity:
            if not command_valid:
                self.get_logger().error(
                    "Rejected velocity target: expected finite ENU values in map frame"
                )
            return

        setpoint = TwistStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = "map"
        setpoint.twist.linear.x = target.twist.linear.x
        setpoint.twist.linear.y = target.twist.linear.y
        setpoint.twist.linear.z = target.twist.linear.z
        self._velocity_pub.publish(setpoint)

    def _state_callback(self, state: State) -> None:
        self._latest_state = state
        self._apply_routing(
            self._router.update_flight_state(self._command_allowed())
        )

    def _watchdog_cycle(self) -> None:
        self._apply_routing(self._router.expire_velocity(time.monotonic()))

    def _apply_routing(self, decision: RoutingDecision) -> None:
        if decision.publish_zero_velocity:
            self._publish_zero_velocity()

    def _publish_zero_velocity(self) -> None:
        setpoint = TwistStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = "map"
        self._velocity_pub.publish(setpoint)


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
