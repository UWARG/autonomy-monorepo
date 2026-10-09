"""Test-only static traffic publisher for the imaginary-obstacle flight test."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from airside_interfaces.msg import TrafficAircraft, TrafficSnapshot
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True, slots=True)
class SyntheticStaticTrafficConfig:
    east_offset_m: float = 0.0
    north_offset_m: float = 20.0
    altitude_agl_m: float = 15.0
    horizontal_keepaway_m: float = 5.0
    vertical_keepaway_m: float = 5.0
    publish_rate_hz: float = 1.0
    include_obstacle: bool = True

    def __post_init__(self) -> None:
        numeric = (
            self.east_offset_m,
            self.north_offset_m,
            self.altitude_agl_m,
            self.horizontal_keepaway_m,
            self.vertical_keepaway_m,
            self.publish_rate_hz,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("synthetic traffic parameters must be finite")
        if self.horizontal_keepaway_m <= 0.0:
            raise ValueError("horizontal keep-away must be positive")
        if self.vertical_keepaway_m <= 0.0:
            raise ValueError("vertical keep-away must be positive")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish rate must be positive")


def offset_coordinate(
    latitude_deg: float,
    longitude_deg: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    """Offset a WGS84 coordinate using the same local approximation as SITL."""

    if not -90.0 < latitude_deg < 90.0:
        raise ValueError("latitude must be between -90 and 90 degrees")
    latitude = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = longitude_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_deg)))
    )
    return latitude, longitude


class SyntheticStaticTrafficNode(Node):
    """Publish one static imaginary aircraft relative to the first armed fix."""

    def __init__(self) -> None:
        super().__init__("synthetic_static_traffic")
        defaults: dict[str, object] = {
            "east_offset_m": 0.0,
            "north_offset_m": 20.0,
            "altitude_agl_m": 15.0,
            "horizontal_keepaway_m": 5.0,
            "vertical_keepaway_m": 5.0,
            "publish_rate_hz": 1.0,
            "include_obstacle": True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._config = SyntheticStaticTrafficConfig(
            east_offset_m=float(self.get_parameter("east_offset_m").value),
            north_offset_m=float(self.get_parameter("north_offset_m").value),
            altitude_agl_m=float(self.get_parameter("altitude_agl_m").value),
            horizontal_keepaway_m=float(
                self.get_parameter("horizontal_keepaway_m").value
            ),
            vertical_keepaway_m=float(
                self.get_parameter("vertical_keepaway_m").value
            ),
            publish_rate_hz=float(self.get_parameter("publish_rate_hz").value),
            include_obstacle=bool(self.get_parameter("include_obstacle").value),
        )
        self._fix: NavSatFix | None = None
        self._relative_altitude_m: float | None = None
        self._state: State | None = None
        self._origin: tuple[float, float] | None = None
        self._sequence = 0
        self._publisher = self.create_publisher(TrafficSnapshot, "/aeac/traffic", 10)
        self.create_subscription(
            NavSatFix,
            "mavros/global_position/global",
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            "mavros/global_position/rel_alt",
            self._altitude_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(State, "mavros/state", self._state_callback, 10)
        self.create_timer(1.0 / self._config.publish_rate_hz, self._publish)
        self.get_logger().warning(
            "SYNTHETIC STATIC TRAFFIC ACTIVE: test data only, not an AEAC server."
        )

    def _fix_callback(self, message: NavSatFix) -> None:
        self._fix = message

    def _altitude_callback(self, message: Float64) -> None:
        self._relative_altitude_m = message.data

    def _state_callback(self, message: State) -> None:
        self._state = message

    def _new_snapshot(self, *, healthy: bool, reason: str) -> TrafficSnapshot:
        message = TrafficSnapshot()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "wgs84"
        message.sequence = self._sequence
        message.connected = True
        message.healthy = healthy
        message.reason = reason
        message.own_aircraft_index = 1
        return message

    def _publish(self) -> None:
        state = self._state
        fix = self._fix
        altitude_m = self._relative_altitude_m
        if state is None or not state.connected:
            self._publisher.publish(
                self._new_snapshot(healthy=False, reason="WAITING_FOR_FCU")
            )
            return
        if not state.armed:
            self._publisher.publish(
                self._new_snapshot(healthy=False, reason="WAITING_FOR_ARM")
            )
            return
        if (
            fix is None
            or altitude_m is None
            or fix.status.status < NavSatStatus.STATUS_FIX
            or not all(
                math.isfinite(value)
                for value in (fix.latitude, fix.longitude, altitude_m)
            )
        ):
            self._publisher.publish(
                self._new_snapshot(healthy=False, reason="WAITING_FOR_POSITION")
            )
            return

        if self._origin is None:
            self._origin = (fix.latitude, fix.longitude)
            self.get_logger().warning(
                "Synthetic origin fixed at first armed position; obstacle is "
                f"{self._config.east_offset_m:.1f}m east / "
                f"{self._config.north_offset_m:.1f}m north."
            )

        self._sequence += 1
        message = self._new_snapshot(healthy=True, reason="")
        own = TrafficAircraft()
        own.aircraft_index = 1
        own.name = "SYNTHETIC-OWN"
        own.latitude_deg = fix.latitude
        own.longitude_deg = fix.longitude
        own.altitude_agl_m = altitude_m
        own.speed_mps = 0.0
        own.heading_deg_true = 0.0
        own.horizontal_keepaway_m = 1.0
        own.vertical_keepaway_m = 1.0
        message.aircraft.append(own)

        if self._config.include_obstacle:
            obstacle_latitude, obstacle_longitude = offset_coordinate(
                self._origin[0],
                self._origin[1],
                self._config.east_offset_m,
                self._config.north_offset_m,
            )
            obstacle = TrafficAircraft()
            obstacle.aircraft_index = 2
            obstacle.name = "SYNTHETIC-STATIC-OBSTACLE"
            obstacle.latitude_deg = obstacle_latitude
            obstacle.longitude_deg = obstacle_longitude
            obstacle.altitude_agl_m = self._config.altitude_agl_m
            obstacle.speed_mps = 0.0
            obstacle.heading_deg_true = 0.0
            obstacle.horizontal_keepaway_m = self._config.horizontal_keepaway_m
            obstacle.vertical_keepaway_m = self._config.vertical_keepaway_m
            message.aircraft.append(obstacle)
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SyntheticStaticTrafficNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
