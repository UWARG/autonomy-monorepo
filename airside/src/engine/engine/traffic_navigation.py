"""Pure conversion from AEAC traffic snapshots to planner obstacles."""

from __future__ import annotations

import math
from dataclasses import dataclass

from obstacle_avoidance import CircleObstacle, ObstacleSnapshot, Point2D

from utils.src.waypoint_utils import east_north_coordinate_offset_m


@dataclass(frozen=True, slots=True)
class TrafficAircraftState:
    aircraft_index: int
    name: str
    latitude_deg: float
    longitude_deg: float
    altitude_agl_m: float
    speed_mps: float
    heading_deg_true: float
    horizontal_keepaway_m: float
    vertical_keepaway_m: float


@dataclass(frozen=True, slots=True)
class TrafficSnapshotState:
    sequence: int
    connected: bool
    healthy: bool
    reason: str
    own_aircraft_index: int
    aircraft: tuple[TrafficAircraftState, ...]
    received_s: float


@dataclass(frozen=True, slots=True)
class TrafficNavigationConfig:
    freshness_s: float = 2.5
    prediction_horizon_s: float = 3.0
    maximum_step_m: float = 1.0
    maximum_predicted_obstacles: int = 512
    buffer_min_m: float = 0.0
    buffer_reaction_s: float = 0.0
    own_speed_assumption_mps: float = 0.0
    consider_all_altitudes: bool = False

    def __post_init__(self) -> None:
        if not math.isfinite(self.freshness_s) or self.freshness_s <= 0.0:
            raise ValueError("traffic freshness must be positive")
        if (
            not math.isfinite(self.prediction_horizon_s)
            or self.prediction_horizon_s < 0.0
        ):
            raise ValueError("prediction horizon must be non-negative")
        if not math.isfinite(self.maximum_step_m) or self.maximum_step_m <= 0.0:
            raise ValueError("maximum prediction step must be positive")
        if self.maximum_predicted_obstacles <= 0:
            raise ValueError("maximum predicted obstacles must be positive")
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in (
                self.buffer_min_m,
                self.buffer_reaction_s,
                self.own_speed_assumption_mps,
            )
        ):
            raise ValueError("traffic buffer parameters must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class TrafficConversion:
    snapshot: ObstacleSnapshot
    reason: str | None
    raw_aircraft_count: int
    relevant_aircraft_count: int
    predicted_circle_count: int
    snapshot_age_s: float = 0.0
    maximum_raw_keepaway_m: float = 0.0
    maximum_buffer_m: float = 0.0
    maximum_effective_radius_m: float = 0.0


def _invalid_conversion(
    *,
    timestamp_s: float,
    reason: str,
    raw_count: int,
    relevant_count: int = 0,
    predicted_count: int = 0,
) -> TrafficConversion:
    return TrafficConversion(
        snapshot=ObstacleSnapshot((), timestamp_s=timestamp_s, healthy=False),
        reason=reason,
        raw_aircraft_count=raw_count,
        relevant_aircraft_count=relevant_count,
        predicted_circle_count=predicted_count,
    )


def traffic_to_obstacle_snapshot(
    *,
    traffic: TrafficSnapshotState,
    now_s: float,
    vehicle_latitude_deg: float,
    vehicle_longitude_deg: float,
    vehicle_east_m: float,
    vehicle_north_m: float,
    vehicle_altitude_agl_m: float,
    goal_altitude_agl_m: float,
    config: TrafficNavigationConfig | None = None,
) -> TrafficConversion:
    """Predict relevant exclusion cylinders in local ENU coordinates."""

    settings = config or TrafficNavigationConfig()
    raw_count = len(traffic.aircraft)
    if not traffic.connected:
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason=traffic.reason or "TRAFFIC_DISCONNECTED",
            raw_count=raw_count,
        )
    if not traffic.healthy:
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason=traffic.reason or "UNHEALTHY_TRAFFIC",
            raw_count=raw_count,
        )
    if traffic.own_aircraft_index < 0:
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason="UNVERIFIED_SELF_IDENTITY",
            raw_count=raw_count,
        )

    age_s = now_s - traffic.received_s
    if not math.isfinite(age_s):
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason="INVALID_TRAFFIC_TIME",
            raw_count=raw_count,
        )
    if age_s < 0.0:
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason="FUTURE_TRAFFIC",
            raw_count=raw_count,
        )
    if age_s > settings.freshness_s:
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason="STALE_TRAFFIC",
            raw_count=raw_count,
        )

    numeric_vehicle = (
        vehicle_latitude_deg,
        vehicle_longitude_deg,
        vehicle_east_m,
        vehicle_north_m,
        vehicle_altitude_agl_m,
        goal_altitude_agl_m,
    )
    if (
        not all(math.isfinite(value) for value in numeric_vehicle)
        or not -90.0 <= vehicle_latitude_deg <= 90.0
        or not -180.0 <= vehicle_longitude_deg <= 180.0
    ):
        return _invalid_conversion(
            timestamp_s=traffic.received_s,
            reason="INVALID_TRAFFIC_REFERENCE",
            raw_count=raw_count,
        )

    vehicle_altitude_min_m = min(vehicle_altitude_agl_m, goal_altitude_agl_m)
    vehicle_altitude_max_m = max(vehicle_altitude_agl_m, goal_altitude_agl_m)
    seen_indices: set[int] = set()
    obstacles: list[CircleObstacle] = []
    relevant_count = 0
    maximum_raw_keepaway_m = 0.0
    maximum_buffer_m = 0.0
    maximum_effective_radius_m = 0.0

    for aircraft in traffic.aircraft:
        if aircraft.aircraft_index in seen_indices:
            return _invalid_conversion(
                timestamp_s=traffic.received_s,
                reason="DUPLICATE_TRAFFIC_IDENTITY",
                raw_count=raw_count,
                relevant_count=relevant_count,
                predicted_count=len(obstacles),
            )
        seen_indices.add(aircraft.aircraft_index)
        if aircraft.aircraft_index == traffic.own_aircraft_index:
            continue

        numeric_aircraft = (
            aircraft.latitude_deg,
            aircraft.longitude_deg,
            aircraft.altitude_agl_m,
            aircraft.speed_mps,
            aircraft.heading_deg_true,
            aircraft.horizontal_keepaway_m,
            aircraft.vertical_keepaway_m,
        )
        if (
            not all(math.isfinite(value) for value in numeric_aircraft)
            or not -90.0 <= aircraft.latitude_deg <= 90.0
            or not -180.0 <= aircraft.longitude_deg <= 180.0
            or aircraft.speed_mps < 0.0
            or not 0.0 <= aircraft.heading_deg_true < 360.0
            or aircraft.horizontal_keepaway_m <= 0.0
            or aircraft.vertical_keepaway_m <= 0.0
        ):
            return _invalid_conversion(
                timestamp_s=traffic.received_s,
                reason="INVALID_TRAFFIC_AIRCRAFT",
                raw_count=raw_count,
                relevant_count=relevant_count,
                predicted_count=len(obstacles),
            )

        traffic_altitude_min_m = aircraft.altitude_agl_m - aircraft.vertical_keepaway_m
        traffic_altitude_max_m = aircraft.altitude_agl_m + aircraft.vertical_keepaway_m
        vertically_relevant = (
            vehicle_altitude_min_m <= traffic_altitude_max_m
            and traffic_altitude_min_m <= vehicle_altitude_max_m
        )
        if not (vertically_relevant or settings.consider_all_altitudes):
            continue
        relevant_count += 1
        buffer_m = (
            max(
                settings.buffer_min_m,
                (aircraft.speed_mps + settings.own_speed_assumption_mps)
                * (age_s + settings.buffer_reaction_s),
            )
            if any(
                value > 0.0
                for value in (
                    settings.buffer_min_m,
                    settings.buffer_reaction_s,
                    settings.own_speed_assumption_mps,
                )
            )
            else 0.0
        )
        effective_radius_m = aircraft.horizontal_keepaway_m + buffer_m
        if not math.isfinite(effective_radius_m):
            return _invalid_conversion(
                timestamp_s=traffic.received_s,
                reason="INVALID_TRAFFIC_AIRCRAFT",
                raw_count=raw_count,
                relevant_count=relevant_count,
            )
        maximum_raw_keepaway_m = max(
            maximum_raw_keepaway_m, aircraft.horizontal_keepaway_m
        )
        maximum_buffer_m = max(maximum_buffer_m, buffer_m)
        maximum_effective_radius_m = max(
            maximum_effective_radius_m, effective_radius_m
        )

        east_offset_m, north_offset_m = east_north_coordinate_offset_m(
            vehicle_latitude_deg,
            vehicle_longitude_deg,
            aircraft.latitude_deg,
            aircraft.longitude_deg,
        )
        heading_rad = math.radians(aircraft.heading_deg_true)
        east_velocity_mps = aircraft.speed_mps * math.sin(heading_rad)
        north_velocity_mps = aircraft.speed_mps * math.cos(heading_rad)
        current_east_m = vehicle_east_m + east_offset_m + east_velocity_mps * age_s
        current_north_m = vehicle_north_m + north_offset_m + north_velocity_mps * age_s

        prediction_distance_m = aircraft.speed_mps * settings.prediction_horizon_s
        if not math.isfinite(prediction_distance_m):
            return _invalid_conversion(
                timestamp_s=traffic.received_s,
                reason="INVALID_TRAFFIC_AIRCRAFT",
                raw_count=raw_count,
                relevant_count=relevant_count,
            )
        maximum_spacing_m = min(settings.maximum_step_m, aircraft.horizontal_keepaway_m)
        segment_count = math.ceil(prediction_distance_m / maximum_spacing_m)
        for segment in range(segment_count + 1):
            future_s = (
                settings.prediction_horizon_s * segment / segment_count
                if segment_count > 0
                else 0.0
            )
            obstacles.append(
                CircleObstacle(
                    center=Point2D(
                        current_east_m + east_velocity_mps * future_s,
                        current_north_m + north_velocity_mps * future_s,
                    ),
                    radius_m=effective_radius_m,
                )
            )
            if len(obstacles) > settings.maximum_predicted_obstacles:
                return _invalid_conversion(
                    timestamp_s=traffic.received_s,
                    reason="TOO_MANY_PREDICTED_OBSTACLES",
                    raw_count=raw_count,
                    relevant_count=relevant_count,
                    predicted_count=len(obstacles),
                )

    return TrafficConversion(
        snapshot=ObstacleSnapshot(
            obstacles=tuple(obstacles),
            timestamp_s=traffic.received_s,
            healthy=True,
        ),
        reason=None,
        raw_aircraft_count=raw_count,
        relevant_aircraft_count=relevant_count,
        predicted_circle_count=len(obstacles),
        snapshot_age_s=age_s,
        maximum_raw_keepaway_m=maximum_raw_keepaway_m,
        maximum_buffer_m=maximum_buffer_m,
        maximum_effective_radius_m=maximum_effective_radius_m,
    )
