"""Turns traffic aircraft into predicted exclusion circles in local ENU."""

from __future__ import annotations

import math

from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .models import CircleObstacle, Point2D, TrafficAircraft


def live_traffic_buffer_m(
    speed_mps: float, age_s: float, own_speed_mps: float = 1.5
) -> float:
    """Extra clearance beyond the server keep-away for live traffic."""

    if not all(
        math.isfinite(value) and value >= 0.0
        for value in (speed_mps, age_s, own_speed_mps)
    ):
        raise ValueError("INVALID_TRAFFIC_BUFFER_INPUT")
    # Motion is already age-compensated and swept over the prediction horizon
    # below. Adding speed * time here would count that displacement twice.
    # Stationary traffic needs only the measured 3 m qualification margin;
    # moving traffic retains the larger mock-validated uncertainty margin.
    return 3.0 if speed_mps <= 0.1 else 5.75


def _valid_aircraft(aircraft: TrafficAircraft) -> bool:
    numbers = (
        aircraft.lat,
        aircraft.lon,
        aircraft.altitude_agl_m,
        aircraft.speed_mps,
        aircraft.heading_deg,
        aircraft.horizontal_keep_away_m,
        aircraft.vertical_keep_away_m,
    )
    return (
        all(math.isfinite(value) for value in numbers)
        and -90.0 <= aircraft.lat <= 90.0
        and -180.0 <= aircraft.lon <= 180.0
        and aircraft.speed_mps >= 0.0
        and 0.0 <= aircraft.heading_deg < 360.0
        and aircraft.horizontal_keep_away_m > 0.0
        and aircraft.vertical_keep_away_m > 0.0
    )


def traffic_to_obstacles(
    traffic: tuple[TrafficAircraft, ...],
    *,
    origin_lat: float,
    origin_lon: float,
    altitude_band_m: tuple[float, float],
    age_s: float,
    prediction_horizon_s: float,
    maximum_step_m: float,
    maximum_obstacles: int,
    live_buffer: bool = False,
    all_altitudes: bool = False,
    own_speed_mps: float = 1.5,
) -> tuple[CircleObstacle, ...]:
    """
    Predicts each aircraft's keep-away cylinder along its track.

    Positions are ENU metres from ``(origin_lat, origin_lon)``. Aircraft whose
    vertical keep-away does not overlap ``altitude_band_m`` are ignored. Each
    remaining aircraft is moved forward by ``age_s`` and then swept over
    ``prediction_horizon_s`` as a chain of circles at most ``maximum_step_m``
    apart.

    Raises ``ValueError`` with a reason code if the traffic cannot be trusted.
    """

    band_min_m, band_max_m = min(altitude_band_m), max(altitude_band_m)
    seen_indices: set[int] = set()
    obstacles: list[CircleObstacle] = []

    for aircraft in traffic:
        if aircraft.aircraft_index in seen_indices:
            raise ValueError("DUPLICATE_TRAFFIC_IDENTITY")
        seen_indices.add(aircraft.aircraft_index)
        if not _valid_aircraft(aircraft):
            raise ValueError("INVALID_TRAFFIC_AIRCRAFT")

        if not all_altitudes and not (
            band_min_m <= aircraft.altitude_agl_m + aircraft.vertical_keep_away_m
            and aircraft.altitude_agl_m - aircraft.vertical_keep_away_m <= band_max_m
        ):
            continue

        east_m, north_m = east_north_coordinate_offset_m(
            origin_lat, origin_lon, aircraft.lat, aircraft.lon
        )
        heading_rad = math.radians(aircraft.heading_deg)
        east_mps = aircraft.speed_mps * math.sin(heading_rad)
        north_mps = aircraft.speed_mps * math.cos(heading_rad)
        east_m += east_mps * age_s
        north_m += north_mps * age_s

        radius_m = aircraft.horizontal_keep_away_m
        if live_buffer:
            radius_m += live_traffic_buffer_m(aircraft.speed_mps, age_s, own_speed_mps)
        spacing_m = min(maximum_step_m, aircraft.horizontal_keep_away_m)
        segments = math.ceil(aircraft.speed_mps * prediction_horizon_s / spacing_m)
        for segment in range(segments + 1):
            future_s = prediction_horizon_s * segment / segments if segments else 0.0
            obstacles.append(
                CircleObstacle(
                    center=Point2D(
                        east_m + east_mps * future_s, north_m + north_mps * future_s
                    ),
                    radius_m=radius_m,
                )
            )
            if len(obstacles) > maximum_obstacles:
                raise ValueError("TOO_MANY_PREDICTED_OBSTACLES")

    return tuple(obstacles)
