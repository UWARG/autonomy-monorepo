"""Find a bounded outward replacement for a blocked lap waypoint."""

from __future__ import annotations

import math

from shapely.geometry import Point, Polygon
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .avoidance import ObstacleAvoider
from .boundary import FlightBoundary
from .clearance import CircularObstacleField
from .models import AvoidanceRequest, PlanStatus, Point2D, TrafficAircraft
from .traffic import traffic_to_obstacles


def replacement_geometry_valid(
    original: Coordinate,
    replacement: Coordinate,
    route: tuple[Coordinate, ...],
    route_index: int,
    boundary: FlightBoundary,
    *,
    max_shift_m: float = 20.0,
    acceptance_radius_m: float = 3.0,
) -> bool:
    """Check a proposal independently of the controller's traffic view."""

    if not (
        3 <= len(route) <= 255
        and 0 <= route_index < len(route)
        and route[route_index] == original
        and replacement.alt == original.alt
    ):
        return False
    local = [
        east_north_coordinate_offset_m(original.lat, original.lon, p.lat, p.lon)
        for p in route
    ]
    polygon = Polygon(local)
    if not polygon.is_valid or polygon.area <= 0:
        return False
    east_m, north_m = east_north_coordinate_offset_m(
        original.lat, original.lon, replacement.lat, replacement.lon
    )
    shift_m = math.hypot(east_m, north_m)
    outward_e, outward_n = -polygon.centroid.x, -polygon.centroid.y
    outward_length_m = math.hypot(outward_e, outward_n)
    return (
        math.isfinite(shift_m)
        and 0.5 <= shift_m <= max_shift_m + 0.01
        and outward_length_m > 0.1
        and (east_m * outward_e + north_m * outward_n) / (shift_m * outward_length_m)
        >= 0.999
        and not polygon.covers(Point(east_m, north_m))
        and boundary.allows_point_with_margin(replacement, acceptance_radius_m)
    )


def outward_replacement(
    *,
    position: Coordinate,
    original: Coordinate,
    route: tuple[Coordinate, ...],
    route_index: int,
    traffic: tuple[TrafficAircraft, ...],
    age_s: float,
    boundary: FlightBoundary,
    avoider: ObstacleAvoider,
    min_shift_m: float = 0.0,
    max_shift_m: int = 20,
    acceptance_radius_m: float = 3.0,
) -> Coordinate | None:
    """Search one-metre outward steps; never change the server keep-away."""

    if not (
        3 <= len(route) <= 255
        and 0 <= route_index < len(route)
        and route[route_index] == original
        and 0 <= age_s <= avoider.config.planner.map_freshness_s
        and math.isfinite(min_shift_m)
        and 0 <= min_shift_m < max_shift_m
    ):
        return None
    local = [
        east_north_coordinate_offset_m(original.lat, original.lon, p.lat, p.lon)
        for p in route
    ]
    polygon = Polygon(local)
    if not polygon.is_valid or polygon.area <= 0:
        return None
    outward_e, outward_n = -polygon.centroid.x, -polygon.centroid.y
    length_m = math.hypot(outward_e, outward_n)
    if length_m < 0.1:
        return None
    direction_e, direction_n = outward_e / length_m, outward_n / length_m
    try:
        obstacles = traffic_to_obstacles(
            traffic,
            origin_lat=position.lat,
            origin_lon=position.lon,
            altitude_band_m=(position.alt, original.alt),
            age_s=age_s,
            prediction_horizon_s=avoider.config.prediction_horizon_s,
            maximum_step_m=avoider.config.maximum_prediction_step_m,
            maximum_obstacles=avoider.config.maximum_predicted_obstacles,
            live_buffer=True,
            all_altitudes=True,
            own_speed_mps=avoider.config.horizontal_speed_mps,
            response_s=avoider.config.live_response_s,
            max_live_speed_mps=avoider.config.maximum_live_traffic_speed_mps,
        )
    except ValueError:
        return None
    field = CircularObstacleField(obstacles)
    required_clearance_m = (
        avoider.config.planner.clearance_margin_m + acceptance_radius_m
    )
    for shift_m in range(max(1, math.floor(min_shift_m + 0.01) + 1), max_shift_m + 1):
        candidate = enu_to_coordinate(
            original, direction_e * shift_m, direction_n * shift_m, original.alt
        )
        if not replacement_geometry_valid(
            original,
            candidate,
            route,
            route_index,
            boundary,
            max_shift_m=max_shift_m,
            acceptance_radius_m=acceptance_radius_m,
        ):
            continue
        candidate_e, candidate_n = east_north_coordinate_offset_m(
            position.lat, position.lon, candidate.lat, candidate.lon
        )
        if (
            field.point_clearance(Point2D(candidate_e, candidate_n))
            < required_clearance_m
        ):
            continue
        plan = ObstacleAvoider(avoider.config).plan(
            AvoidanceRequest(
                position=position,
                goal=candidate,
                traffic=traffic,
                traffic_age_s=age_s,
                required_waypoint=candidate,
            )
        )
        if (
            plan.status is PlanStatus.PATH_FOUND
            and plan.next_waypoint is not None
            and boundary.allows_segment(position, plan.next_waypoint)
        ):
            return candidate
    return None
