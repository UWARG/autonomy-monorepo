from __future__ import annotations

import math

import py_trees
import rclpy.node
from engine.constants import WAYPOINT_ACCEPTANCE_RADIUS_M
from utils.src.waypoint_utils import east_north_coordinate_offset_m
from mavros_msgs.msg import GlobalPositionTarget, State
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


_SETPOINT_TOPIC = "/mavros/setpoint_raw/global"
_GLOBAL_POSITION_TOPIC = "/mavros/global_position/global"
_REL_ALT_TOPIC = "/mavros/global_position/rel_alt"
_STATE_TOPIC = "/mavros/state"

_GUIDED_MODE = "GUIDED"

# Position-only setpoint: ignores velocity, acceleration and yaw fields
_TYPE_MASK = (
    GlobalPositionTarget.IGNORE_VX
    | GlobalPositionTarget.IGNORE_VY
    | GlobalPositionTarget.IGNORE_VZ
    | GlobalPositionTarget.IGNORE_AFX
    | GlobalPositionTarget.IGNORE_AFY
    | GlobalPositionTarget.IGNORE_AFZ
    | GlobalPositionTarget.IGNORE_YAW
    | GlobalPositionTarget.IGNORE_YAW_RATE
)


class ReturnToLaunch(py_trees.behaviour.Behaviour):
    """
    Return to the launch site.
    """

    def __init__(self) -> None:
        super().__init__(name="ReturnToLaunch")

        self.blackboard = self.attach_blackboard_client(name="ReturnToLaunch")
        self.blackboard.register_key(key="latitude", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="longitude", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="altitude", access=py_trees.common.Access.READ)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._setpoint: GlobalPositionTarget | None = None

        self._setpoint_pub = self._node.create_publisher(
            msg_type=GlobalPositionTarget,
            topic=_SETPOINT_TOPIC,
            qos_profile=10,
        )

        self._fix_sub = self._node.create_subscription(
            msg_type=NavSatFix,
            topic=_GLOBAL_POSITION_TOPIC,
            callback=self._fix_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._rel_alt_sub = self._node.create_subscription(
            msg_type=Float64,
            topic=_REL_ALT_TOPIC,
            callback=self._rel_alt_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=_STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._latest_rel_alt_m = msg.data

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def initialise(self) -> None:
        latitude = self.blackboard.get("latitude")
        longitude = self.blackboard.get("longitude")
        altitude = self.blackboard.get("altitude")

        self._setpoint = GlobalPositionTarget()
        self._setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        self._setpoint.type_mask = _TYPE_MASK
        self._setpoint.latitude = latitude
        self._setpoint.longitude = longitude
        self._setpoint.altitude = altitude


    def update(self) -> py_trees.common.Status:
        if self._setpoint is None:
            self._node.get_logger().error(f"{self.name}: no waypoint to fly to")
            return py_trees.common.Status.FAILURE

        if (
            self._latest_state is None
            or self._latest_fix is None
            or self._latest_rel_alt_m is None
        ):
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{_STATE_TOPIC}', "
                f"'{_GLOBAL_POSITION_TOPIC}' and '{_REL_ALT_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if self._latest_state.mode != _GUIDED_MODE:
            self._node.get_logger().warning(
                f"{self.name}: flight controller in '{self._latest_state.mode}' "
                f"mode, not '{_GUIDED_MODE}' - holding off on setpoints",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        self._setpoint.header.stamp = self._node.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._setpoint)

        east_m, north_m = east_north_coordinate_offset_m(
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            self._setpoint.latitude,
            self._setpoint.longitude,
        )
        up_m = self._setpoint.altitude - self._latest_rel_alt_m
        distance = math.sqrt(east_m**2 + north_m**2 + up_m**2)

        if distance <= WAYPOINT_ACCEPTANCE_RADIUS_M:
            self._node.get_logger().info(
                f"{self.name}: reached waypoint ({distance:.2f}m away)"
            )
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: {distance:.2f}m to waypoint",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._setpoint = None