"""
Single point of contact between the engine and MAVROS position setpoints.

Behaviors publish where they want the drone to go on ``position_controller/target``;
this node turns each target into a MAVROS guided-mode global setpoint. On the
way it routes the path around the other aircraft reported on
``position_controller/obstacle``.
"""

from __future__ import annotations

import math

import rclpy
import rclpy.node
from airside_interfaces.msg import Coordinate, Obstacle
from mavros_msgs.msg import GlobalPositionTarget
from navigation.visibility_graph import (
    KeepAwayZone,
    PlanStatus,
    Point,
    VisibilityGraphPlanner,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from utils.src.constants import EARTH_RADIUS_M
from utils.src.waypoint_utils import east_north_coordinate_offset_m


def _offset_coordinate(
    from_lat: float, from_lon: float, east_m: float, north_m: float
) -> tuple[float, float]:
    """
    Inverse of ``east_north_coordinate_offset_m``: the ``(lat, lon)`` that lies
    ``(east_m, north_m)`` away from a point.
    """

    lat = from_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = from_lon + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians((from_lat + lat) / 2.0)))
    )
    return lat, lon


class PositionController(rclpy.node.Node):
    """
    Forwards every ``Coordinate`` target (lat, lon, relative alt) to
    ``mavros/setpoint_raw/global`` as a position-only setpoint, replacing it
    with the first point of a detour whenever the way there is blocked by the
    keep-away zone of an obstacle.

    Obstacles are kept per ``aircraft_index``. Each one stays frozen where it
    was last reported until the next report for that index replaces it.
    """

    TARGET_TOPIC = "position_controller/target"
    OBSTACLE_TOPIC = "position_controller/obstacle"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
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

        self._obstacles: dict[int, Obstacle] = {}
        self._latest_fix: NavSatFix | None = None
        self._planner = VisibilityGraphPlanner()

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
        self._obstacle_sub = self.create_subscription(
            msg_type=Obstacle,
            topic=self.OBSTACLE_TOPIC,
            callback=self._obstacle_callback,
            qos_profile=10,
        )
        self._fix_sub = self.create_subscription(
            msg_type=NavSatFix,
            topic=self.GLOBAL_POSITION_TOPIC,
            callback=self._fix_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Position controller ready - forwarding '{self.TARGET_TOPIC}' "
            f"to '{self.SETPOINT_TOPIC}', avoiding '{self.OBSTACLE_TOPIC}'."
        )

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _obstacle_callback(self, obstacle: Obstacle) -> None:
        values = (
            obstacle.position.lat,
            obstacle.position.lon,
            obstacle.horizontal_keep_away,
            obstacle.speed,
            obstacle.direction,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                f"Ignoring obstacle {obstacle.aircraft_index} with non-finite fields",
                throttle_duration_sec=5.0,
            )
            return

        self._obstacles[obstacle.aircraft_index] = obstacle

    def _avoid_obstacles(self, target: Coordinate) -> Coordinate:
        """
        Returns ``target``, or the point to fly to instead to stay out of every
        obstacle's keep-away zone on the way there.
        """

        if not self._obstacles:
            return target

        if self._latest_fix is None:
            self.get_logger().warning(
                f"No position on '{self.GLOBAL_POSITION_TOPIC}' yet - "
                "forwarding targets without obstacle avoidance",
                throttle_duration_sec=5.0,
            )
            return target

        # Flat frame in meters, centered on the drone
        origin_lat = self._latest_fix.latitude
        origin_lon = self._latest_fix.longitude

        def to_local(lat: float, lon: float) -> Point:
            return east_north_coordinate_offset_m(origin_lat, origin_lon, lat, lon)

        zones = [
            KeepAwayZone.from_obstacle(
                position=to_local(obstacle.position.lat, obstacle.position.lon),
                horizontal_keep_away_m=obstacle.horizontal_keep_away,
                speed_mps=obstacle.speed,
                direction_deg=obstacle.direction,
            )
            for obstacle in self._obstacles.values()
        ]
        plan = self._planner.plan(
            position=(0.0, 0.0),
            destination=to_local(target.lat, target.lon),
            zones=zones,
        )
        if plan.status == PlanStatus.DIRECT:
            return target

        lat, lon = _offset_coordinate(origin_lat, origin_lon, *plan.point)
        self.get_logger().info(
            f"Obstacle avoidance: {plan.status.value} via "
            f"lat={lat:.7f} lon={lon:.7f}",
            throttle_duration_sec=2.0,
        )
        return Coordinate(lat=lat, lon=lon, alt=target.alt)

    def _target_callback(self, target: Coordinate) -> None:
        target = self._avoid_obstacles(target)

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
