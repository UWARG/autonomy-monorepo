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
from utils.src.waypoint_utils import east_north_coordinate_offset_m
from airside_interfaces.msg import Coordinate
from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


_POSITION_TARGET_TOPIC = "position_controller/target"
_GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_STATE_TOPIC = "mavros/state"


class FlyToWaypoint(py_trees.behaviour.Behaviour):
    """
    Flies the drone to ``current_waypoint`` by publishing it as a position target
    for the ``position_controller`` node.

    Returns RUNNING while traveling, SUCCESS once within ``WAYPOINT_ACCEPTANCE_RADIUS_M``
    of the waypoint, and FAILURE if the waypoint is not reached within
    ``WAYPOINT_NAV_TIMEOUT_S``.
    """


    def __init__(self, name: str = "FlyToWaypoint") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._target: Coordinate | None = None
        self._start_time_s = 0.0

        self._target_pub = self._node.create_publisher(
            msg_type=Coordinate,
            topic=_POSITION_TARGET_TOPIC,
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

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def initialise(self) -> None:
        try:
            waypoint = self.blackboard.get(blackboard_keys.CURRENT_WAYPOINT)
        except KeyError:
            self._target = None
            return

        self._target = Coordinate(lat=waypoint.lat, lon=waypoint.lon, alt=waypoint.alt)

        self._start_time_s = self._now_s()
        self._node.get_logger().info(f"{self.name}: flying to {waypoint}")

    def update(self) -> py_trees.common.Status:
        if self._target is None:
            self._node.get_logger().error(f"{self.name}: no waypoint to fly to")
            return py_trees.common.Status.FAILURE

        if self._now_s() - self._start_time_s > WAYPOINT_NAV_TIMEOUT_S:
            self._node.get_logger().error(
                f"{self.name}: waypoint not reached within {WAYPOINT_NAV_TIMEOUT_S}s"
            )
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

        if self._latest_state.mode != GUIDED_MODE:
            self._node.get_logger().warning(
                f"{self.name}: flight controller in '{self._latest_state.mode}' "
                f"mode, not '{GUIDED_MODE}' - holding off on position targets",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        self._target_pub.publish(self._target)

        east_m, north_m = east_north_coordinate_offset_m(
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            self._target.lat,
            self._target.lon,
        )
        up_m = self._target.alt - self._latest_rel_alt_m
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
            self._target = None
