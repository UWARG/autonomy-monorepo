from __future__ import annotations

import math
import os

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import (
    GUIDED_MODE,
    WAYPOINT_ACCEPTANCE_RADIUS_M,
    WAYPOINT_NAV_TIMEOUT_S,
    WAYPOINT_PASS_RADIUS_M,
)
from utils.src.waypoint_utils import east_north_coordinate_offset_m
from airside_interfaces.msg import Coordinate, PositionTarget, WaypointAdjustment
from mavros_msgs.msg import State
from obstacle_avoidance.boundary import FlightBoundary
from obstacle_avoidance.waypoint_relocation import replacement_geometry_valid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


_POSITION_TARGET_TOPIC = "position_controller/target"
_GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_STATE_TOPIC = "mavros/state"
_ADJUSTMENT_TOPIC = "position_controller/waypoint_adjustment"


class FlyToWaypoint(py_trees.behaviour.Behaviour):
    """
    Flies the drone to ``current_waypoint`` by publishing it as a position target
    for the ``position_controller`` node.

    Returns RUNNING while traveling, SUCCESS once within ``acceptance_radius_m``
    of the waypoint, and FAILURE if the waypoint is not reached within
    ``WAYPOINT_NAV_TIMEOUT_S``.

    With ``fly_through``, ``waypoint_path`` is sent along for smooth pathing.
    """

    def __init__(
        self,
        name: str = "FlyToWaypoint",
        acceptance_radius_m: float = WAYPOINT_ACCEPTANCE_RADIUS_M,
        fly_through: bool = False,
    ) -> None:
        super().__init__(name=name)
        self._acceptance_radius_m = acceptance_radius_m
        self._fly_through = fly_through

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.READ
        )
        if fly_through:
            self.blackboard.register_key(
                key=blackboard_keys.WAYPOINT_PATH,
                access=py_trees.common.Access.READ,
            )
            self.blackboard.register_key(
                key=blackboard_keys.WAYPOINTS, access=py_trees.common.Access.READ
            )
            self.blackboard.register_key(
                key=blackboard_keys.WAYPOINT_INDEX,
                access=py_trees.common.Access.READ,
            )
        self._leg_id_counter = 0

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._target: PositionTarget | None = None
        # Where the drone was when the leg started, for the abeam check.
        self._leg_start: NavSatFix | None = None
        self._start_time_s = 0.0
        self._original_waypoint: Coordinate | None = None
        self._replacement_active = False
        self._last_adjustment_sequence = 0
        self._boundary: FlightBoundary | None = None
        boundary_raw = os.environ.get("FLIGHT_BOUNDARY_WGS84", "").strip()
        if boundary_raw:
            try:
                self._boundary = FlightBoundary.from_json(boundary_raw)
            except ValueError:
                pass

        self._target_pub = self._node.create_publisher(
            msg_type=PositionTarget,
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
        self._adjustment_sub = self._node.create_subscription(
            WaypointAdjustment, _ADJUSTMENT_TOPIC, self._adjustment_callback, 10
        )

    def _adjustment_callback(self, msg: WaypointAdjustment) -> None:
        target = self._target
        if (
            not self._fly_through
            or target is None
            or self._original_waypoint is None
            or self._boundary is None
            or msg.leg_id == 0
            or msg.leg_id != target.leg_id
            or msg.route_index != target.route_index
            or msg.traffic_sequence <= self._last_adjustment_sequence
            or msg.original != self._original_waypoint
            or target.target_index >= len(target.path)
        ):
            return
        route = tuple(
            Coordinate(lat=p.lat, lon=p.lon, alt=p.alt) for p in target.route_vertices
        )
        replacement = msg.replacement
        if not replacement_geometry_valid(
            self._original_waypoint,
            replacement,
            route,
            target.route_index,
            self._boundary,
        ):
            return
        target.target = Coordinate(
            lat=replacement.lat, lon=replacement.lon, alt=replacement.alt
        )
        target.path[target.target_index] = target.target
        self._replacement_active = True
        self._last_adjustment_sequence = msg.traffic_sequence
        self._node.get_logger().warning(
            f"{self.name}: replaced blocked waypoint on leg {target.leg_id}"
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

        self._target = PositionTarget(
            target=Coordinate(lat=waypoint.lat, lon=waypoint.lon, alt=waypoint.alt)
        )
        self._original_waypoint = Coordinate(
            lat=waypoint.lat, lon=waypoint.lon, alt=waypoint.alt
        )
        self._replacement_active = False
        self._last_adjustment_sequence = 0
        if self._fly_through:
            try:
                path, target_index = self.blackboard.get(blackboard_keys.WAYPOINT_PATH)
            except KeyError:
                path, target_index = [], 0
            self._target.path = [
                Coordinate(lat=point.lat, lon=point.lon, alt=point.alt)
                for point in path
            ]
            self._target.target_index = target_index
            try:
                route = self.blackboard.get(blackboard_keys.WAYPOINTS)
                waypoint_index = self.blackboard.get(blackboard_keys.WAYPOINT_INDEX)
            except KeyError:
                route, waypoint_index = [], 0
            if route and 3 <= len(route) <= 255 and waypoint_index > 0:
                self._leg_id_counter += 1
                self._target.leg_id = self._leg_id_counter
                self._target.route_index = (waypoint_index - 1) % len(route)
                self._target.route_vertices = [
                    Coordinate(lat=p.lat, lon=p.lon, alt=p.alt) for p in route
                ]

        self._leg_start = self._latest_fix
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

        waypoint = self._target.target
        east_m, north_m = east_north_coordinate_offset_m(
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            waypoint.lat,
            waypoint.lon,
        )
        up_m = waypoint.alt - self._latest_rel_alt_m
        distance = math.sqrt(east_m**2 + north_m**2 + up_m**2)

        if distance <= self._acceptance_radius_m:
            self._node.get_logger().info(
                f"{self.name}: reached waypoint ({distance:.2f}m away)"
            )
            return py_trees.common.Status.SUCCESS

        if (
            self._fly_through
            and not self._replacement_active
            and self._passed_abeam(east_m, north_m)
        ):
            self._node.get_logger().info(
                f"{self.name}: passed abeam of waypoint ({distance:.2f}m away)"
            )
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: {distance:.2f}m to waypoint",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING

    def _passed_abeam(self, east_m: float, north_m: float) -> bool:
        """
        Whether the drone, ``(east_m, north_m)`` short of the waypoint, has
        crossed the line through it perpendicular to the leg, close enough to
        count.
        """

        if (
            self._leg_start is None
            or math.hypot(east_m, north_m) > WAYPOINT_PASS_RADIUS_M
        ):
            return False
        waypoint = self._target.target
        leg_east_m, leg_north_m = east_north_coordinate_offset_m(
            self._leg_start.latitude,
            self._leg_start.longitude,
            waypoint.lat,
            waypoint.lon,
        )
        # The waypoint is behind the drone along the leg direction.
        return east_m * leg_east_m + north_m * leg_north_m <= 0.0

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._target = None
