"""Single, fail-closed owner of Airside global position setpoints."""

from __future__ import annotations

import math
import time

import rclpy
import rclpy.node
from airside_interfaces.msg import Coordinate, Obstacle, ObstacleSnapshotStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import GlobalPositionTarget, State
from navigation.traffic_store import ObstacleRecord, TrafficStore, altitude_ranges_overlap
from navigation.visibility_graph import KeepAwayZone, PlanStatus, Point, VisibilityGraphPlanner
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from utils.src.constants import EARTH_RADIUS_M
from utils.src.waypoint_utils import east_north_coordinate_offset_m


def _offset_coordinate(
    from_lat: float, from_lon: float, east_m: float, north_m: float
) -> tuple[float, float]:
    lat = from_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = from_lon + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians((from_lat + lat) / 2.0)))
    )
    return lat, lon


class PositionController(rclpy.node.Node):
    TARGET_TOPIC = "position_controller/target"
    OBSTACLE_TOPIC = "position_controller/obstacle"
    SNAPSHOT_TOPIC = "position_controller/obstacle_snapshot"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
    RELATIVE_ALTITUDE_TOPIC = "mavros/global_position/rel_alt"
    STATE_TOPIC = "mavros/state"
    SETPOINT_TOPIC = "mavros/setpoint_raw/global"
    DIAGNOSTICS_TOPIC = "position_controller/diagnostics"
    GUIDED_MODE = "GUIDED"

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
        defaults: dict[str, object] = {
            "traffic_required": False,
            "own_aircraft_index": -1,
            "traffic_freshness_s": 2.5,
            "telemetry_freshness_s": 1.0,
            "target_freshness_s": 1.0,
            "control_rate_hz": 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._traffic_required = bool(self.get_parameter("traffic_required").value)
        self._telemetry_freshness_s = float(
            self.get_parameter("telemetry_freshness_s").value
        )
        self._target_freshness_s = float(self.get_parameter("target_freshness_s").value)
        own_index = int(self.get_parameter("own_aircraft_index").value)
        self._traffic_store: TrafficStore | None = None
        if self._traffic_required:
            try:
                self._traffic_store = TrafficStore(
                    own_aircraft_index=own_index,
                    freshness_s=float(self.get_parameter("traffic_freshness_s").value),
                )
            except ValueError as error:
                self.get_logger().error(f"Live traffic configuration invalid: {error}")

        self._latest_fix: NavSatFix | None = None
        self._latest_altitude: Float64 | None = None
        self._latest_state: State | None = None
        self._fix_received_s: float | None = None
        self._altitude_received_s: float | None = None
        self._state_received_s: float | None = None
        self._target: Coordinate | None = None
        self._target_received_s: float | None = None
        self._hold_reason: str | None = None
        self._planner = VisibilityGraphPlanner()

        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, self.SETPOINT_TOPIC, 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.DIAGNOSTICS_TOPIC, 10
        )
        self.create_subscription(Coordinate, self.TARGET_TOPIC, self._target_callback, 10)
        self.create_subscription(Obstacle, self.OBSTACLE_TOPIC, self._obstacle_callback, 10)
        self.create_subscription(
            ObstacleSnapshotStatus,
            self.SNAPSHOT_TOPIC,
            self._snapshot_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            self.GLOBAL_POSITION_TOPIC,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            self.RELATIVE_ALTITUDE_TOPIC,
            self._altitude_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(State, self.STATE_TOPIC, self._state_callback, 10)
        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._control_cycle)

    def _fix_callback(self, message: NavSatFix) -> None:
        self._latest_fix = message
        self._fix_received_s = time.monotonic()

    def _altitude_callback(self, message: Float64) -> None:
        self._latest_altitude = message
        self._altitude_received_s = time.monotonic()

    def _state_callback(self, message: State) -> None:
        self._latest_state = message
        self._state_received_s = time.monotonic()
        if not self._command_allowed():
            self._target = None
            self._target_received_s = None
            self._hold_reason = None
            self._planner = VisibilityGraphPlanner()

    def _target_callback(self, target: Coordinate) -> None:
        # A target sent while the pilot owns the FCU must not become an
        # autonomous command when GUIDED is selected again.
        if not self._command_allowed():
            return
        values = (target.lat, target.lon, target.alt)
        if (
            not all(math.isfinite(value) for value in values)
            or not -90.0 <= target.lat <= 90.0
            or not -180.0 <= target.lon <= 180.0
        ):
            self._target = None
            self._target_received_s = None
            self._hold_reason = "INVALID_TARGET_HOLD"
            self._publish_diagnostic("INVALID_TARGET", ready=False)
            return
        self._target = target
        self._target_received_s = time.monotonic()
        self._hold_reason = None

    def _obstacle_callback(self, obstacle: Obstacle) -> None:
        if self._traffic_store is None:
            return
        values = (
            obstacle.position.lat,
            obstacle.position.lon,
            obstacle.position.alt,
            obstacle.horizontal_keep_away,
            obstacle.vertical_keep_away,
            obstacle.speed,
            obstacle.direction,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or not -90.0 <= obstacle.position.lat <= 90.0
            or not -180.0 <= obstacle.position.lon <= 180.0
            or obstacle.horizontal_keep_away <= 0.0
            or obstacle.vertical_keep_away <= 0.0
            or obstacle.speed < 0.0
            or not 0.0 <= obstacle.direction < 360.0
        ):
            return
        self._traffic_store.receive_obstacle(
            ObstacleRecord(
                sequence=obstacle.sequence,
                aircraft_index=obstacle.aircraft_index,
                latitude_deg=obstacle.position.lat,
                longitude_deg=obstacle.position.lon,
                altitude_agl_m=obstacle.position.alt,
                horizontal_keep_away_m=obstacle.horizontal_keep_away,
                vertical_keep_away_m=obstacle.vertical_keep_away,
                speed_mps=obstacle.speed,
                direction_deg=obstacle.direction,
            )
        )

    def _snapshot_callback(self, status: ObstacleSnapshotStatus) -> None:
        if self._traffic_store is None:
            return
        self._traffic_store.receive_status(
            sequence=status.sequence,
            connected=status.connected,
            healthy=status.healthy,
            reason=status.reason,
            aircraft_indices=tuple(status.aircraft_indices),
            received_s=time.monotonic(),
        )

    def _command_allowed(self) -> bool:
        return bool(
            self._latest_state is not None
            and self._latest_state.connected
            and self._latest_state.armed
            and self._latest_state.mode == self.GUIDED_MODE
        )

    def _navigation_state(self, now_s: float) -> str | None:
        if self._traffic_required and self._traffic_store is None:
            return "INVALID_LIVE_TRAFFIC_CONFIGURATION"
        if (
            self._latest_fix is None
            or self._latest_altitude is None
            or self._latest_state is None
            or self._fix_received_s is None
            or self._altitude_received_s is None
            or self._state_received_s is None
        ):
            return "WAITING_FOR_NAVIGATION_DATA"
        if (
            self._latest_fix.status.status < NavSatStatus.STATUS_FIX
            or not all(
                math.isfinite(value)
                for value in (
                    self._latest_fix.latitude,
                    self._latest_fix.longitude,
                    self._latest_altitude.data,
                )
            )
            or not -90.0 <= self._latest_fix.latitude <= 90.0
            or not -180.0 <= self._latest_fix.longitude <= 180.0
        ):
            return "INVALID_NAVIGATION_DATA"
        fix_stamp_s = (
            self._latest_fix.header.stamp.sec
            + self._latest_fix.header.stamp.nanosec / 1e9
        )
        fix_age_s = self.get_clock().now().nanoseconds / 1e9 - fix_stamp_s
        if fix_stamp_s <= 0.0 or fix_age_s < -1.0 or fix_age_s > self._telemetry_freshness_s:
            return "STALE_GPS_MEASUREMENT"
        if max(
            now_s - self._fix_received_s,
            now_s - self._altitude_received_s,
            now_s - self._state_received_s,
        ) > self._telemetry_freshness_s:
            return "STALE_NAVIGATION_DATA"
        return None

    def _traffic(self, now_s: float) -> tuple[tuple[ObstacleRecord, ...], str | None]:
        if not self._traffic_required:
            return (), None
        assert self._traffic_store is not None
        return self._traffic_store.current(now_s)

    def _vertically_relevant(self, obstacle: ObstacleRecord, target_altitude: float) -> bool:
        assert self._latest_altitude is not None
        return altitude_ranges_overlap(
            vehicle_altitude_agl_m=self._latest_altitude.data,
            target_altitude_agl_m=target_altitude,
            obstacle_altitude_agl_m=obstacle.altitude_agl_m,
            vertical_keep_away_m=obstacle.vertical_keep_away_m,
        )

    def _planned_target(
        self, target: Coordinate, obstacles: tuple[ObstacleRecord, ...]
    ) -> tuple[Coordinate, str]:
        assert self._latest_fix is not None and self._latest_altitude is not None
        origin_lat = self._latest_fix.latitude
        origin_lon = self._latest_fix.longitude

        def to_local(lat: float, lon: float) -> Point:
            return east_north_coordinate_offset_m(origin_lat, origin_lon, lat, lon)

        zones = [
            KeepAwayZone.from_obstacle(
                position=to_local(item.latitude_deg, item.longitude_deg),
                horizontal_keep_away_m=item.horizontal_keep_away_m,
                speed_mps=item.speed_mps,
                direction_deg=item.direction_deg,
            )
            for item in obstacles
            if self._vertically_relevant(item, target.alt)
        ]
        plan = self._planner.plan((0.0, 0.0), to_local(target.lat, target.lon), zones)
        if plan.status == PlanStatus.DIRECT:
            return target, plan.status.value
        lat, lon = _offset_coordinate(origin_lat, origin_lon, *plan.point)
        altitude = self._latest_altitude.data if plan.status == PlanStatus.HOLD else target.alt
        return Coordinate(lat=lat, lon=lon, alt=altitude), plan.status.value

    def _hold_target(self) -> Coordinate:
        assert self._latest_fix is not None and self._latest_altitude is not None
        return Coordinate(
            lat=self._latest_fix.latitude,
            lon=self._latest_fix.longitude,
            alt=self._latest_altitude.data,
        )

    def _control_cycle(self) -> None:
        if self._target is None and self._hold_reason is None:
            return
        if not self._command_allowed():
            self._publish_diagnostic("PILOT_OR_FCU_CONTROL", ready=False)
            return
        now_s = time.monotonic()
        navigation_reason = self._navigation_state(now_s)
        if navigation_reason is not None:
            self._publish_diagnostic(navigation_reason, ready=False)
            return
        if self._hold_reason is not None:
            self._publish_setpoint(self._hold_target())
            self._publish_diagnostic(self._hold_reason, ready=False)
            return
        assert self._target is not None and self._target_received_s is not None
        if now_s - self._target_received_s > self._target_freshness_s:
            self._publish_setpoint(self._hold_target())
            self._publish_diagnostic("STALE_TARGET_HOLD", ready=False)
            return
        obstacles, traffic_reason = self._traffic(now_s)
        if traffic_reason is not None:
            self._publish_setpoint(self._hold_target())
            self._publish_diagnostic(traffic_reason, ready=False)
            return
        planned, status = self._planned_target(self._target, obstacles)
        self._publish_setpoint(planned)
        self._publish_diagnostic(status, ready=status != PlanStatus.HOLD.value)

    def _publish_setpoint(self, target: Coordinate) -> None:
        setpoint = GlobalPositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        setpoint.type_mask = self.TYPE_MASK
        setpoint.latitude = target.lat
        setpoint.longitude = target.lon
        setpoint.altitude = target.alt
        self._setpoint_pub.publish(setpoint)

    def _publish_diagnostic(self, reason: str, *, ready: bool) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "position_controller/live_traffic"
        status.hardware_id = "airside"
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        status.message = reason
        status.values = [KeyValue(key="ready", value=str(ready).lower())]
        message.status = [status]
        self._diagnostics_pub.publish(message)


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
