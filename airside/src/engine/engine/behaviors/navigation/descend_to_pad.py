from __future__ import annotations

import math

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import (
    GUIDED_MODE,
    WAYPOINT_ACCEPTANCE_RADIUS_M,
    WAYPOINT_NAV_TIMEOUT_S,
)
from engine.ground_log import send_to_ground
from utils.src.waypoint_utils import east_north_coordinate_offset_m
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandTOL
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


_SETPOINT_TOPIC = "mavros/setpoint_raw/global"
_GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_STATE_TOPIC = "mavros/state"
_LAND_SERVICE = "mavros/cmd/land"

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


class DescendToLandingPad(py_trees.behaviour.Behaviour):
    """
    Flies to the claimed landing pad in guided mode, then commands a LAND-mode
    descent onto it.

    Reads ``target_landing_pad``. First publishes MAVROS guided-mode setpoints
    to fly to the pad's global coordinate, returning RUNNING while traveling
    and FAILURE if the pad is not reached within ``WAYPOINT_NAV_TIMEOUT_S``.
    Once within ``WAYPOINT_ACCEPTANCE_RADIUS_M`` of the pad, calls the MAVROS
    land service and returns SUCCESS once the descent is accepted. The drone's
    touchdown is confirmed separately by ``TouchingGround``. Returns FAILURE if
    the land command is rejected.
    """

    def __init__(self, name: str = "DescendToLandingPad") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.TARGET_LANDING_PAD, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._setpoint: GlobalPositionTarget | None = None
        self._start_time_s = 0.0
        self._reached_pad = False
        self._land_future = None
        self._land_accepted = False

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
        self._land_client = self._node.create_client(
            srv_type=CommandTOL, srv_name=_LAND_SERVICE
        )

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._latest_rel_alt_m = msg.data

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def initialise(self) -> None:
        try:
            pad = self.blackboard.get(blackboard_keys.TARGET_LANDING_PAD)
        except KeyError:
            self._setpoint = None
            return

        self._setpoint = GlobalPositionTarget()
        self._setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        self._setpoint.type_mask = _TYPE_MASK
        self._setpoint.latitude = pad.lat
        self._setpoint.longitude = pad.lon
        self._setpoint.altitude = pad.alt

        self._start_time_s = self._now_s()
        self._reached_pad = False
        self._land_future = None
        self._land_accepted = False
        self._node.get_logger().info(f"{self.name}: descending to pad {pad}")

    def update(self) -> py_trees.common.Status:
        if self._setpoint is None:
            self._node.get_logger().error(f"{self.name}: no landing pad to descend to")
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

        if not self._reached_pad:
            return self._fly_to_pad()

        return self._command_land()

    def _fly_to_pad(self) -> py_trees.common.Status:
        """Publish guided-mode setpoints until the pad is reached."""

        if self._now_s() - self._start_time_s > WAYPOINT_NAV_TIMEOUT_S:
            self._node.get_logger().error(
                f"{self.name}: pad not reached within {WAYPOINT_NAV_TIMEOUT_S}s"
            )
            return py_trees.common.Status.FAILURE

        if self._latest_state.mode != GUIDED_MODE:
            self._node.get_logger().warning(
                f"{self.name}: flight controller in '{self._latest_state.mode}' "
                f"mode, not '{GUIDED_MODE}' - holding off on setpoints",
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
                f"{self.name}: over the pad ({distance:.2f}m away), commanding land"
            )
            send_to_ground(self._node, "ENG: over pad, landing to drop")
            self._reached_pad = True
            return py_trees.common.Status.RUNNING

        self._node.get_logger().info(
            f"{self.name}: {distance:.2f}m to pad",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING

    def _command_land(self) -> py_trees.common.Status:
        """Send the land service call and track its response."""

        if self._land_accepted:
            return py_trees.common.Status.SUCCESS

        if self._land_future is None:
            if not self._land_client.service_is_ready():
                self._node.get_logger().warning(
                    f"{self.name}: waiting for '{_LAND_SERVICE}' service",
                    throttle_duration_sec=5.0,
                )
                return py_trees.common.Status.RUNNING

            self._land_future = self._land_client.call_async(CommandTOL.Request())
            self._node.get_logger().info(f"{self.name}: commanding land onto pad")
            return py_trees.common.Status.RUNNING

        if not self._land_future.done():
            return py_trees.common.Status.RUNNING

        response = self._land_future.result()
        self._land_future = None
        if response is None or not response.success:
            self._node.get_logger().error(
                f"{self.name}: land command rejected: {response}"
            )
            return py_trees.common.Status.FAILURE

        self._land_accepted = True
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._setpoint = None
            self._reached_pad = False
            self._land_future = None
            self._land_accepted = False
