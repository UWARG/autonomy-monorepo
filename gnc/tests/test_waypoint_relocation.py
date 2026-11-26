"""A blocked lap point may move outward, never beyond the approved fence."""

import json
import math

from obstacle_avoidance import ObstacleAvoider, TrafficAircraft
from obstacle_avoidance.boundary import FlightBoundary
from obstacle_avoidance.waypoint_relocation import (
    outward_replacement,
    replacement_geometry_valid,
)
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

HOME = Coordinate(43.0, -80.0, 10.0)
ROUTE = tuple(
    enu_to_coordinate(HOME, east, north, 10.0)
    for east, north in ((10, 10), (10, -10), (-10, -10), (-10, 10))
)


def boundary(half_width_m: float) -> FlightBoundary:
    points = [
        enu_to_coordinate(HOME, east, north)
        for east, north in (
            (-half_width_m, -half_width_m),
            (half_width_m, -half_width_m),
            (half_width_m, half_width_m),
            (-half_width_m, half_width_m),
        )
    ]
    return FlightBoundary.from_json(
        json.dumps([[point.lat, point.lon] for point in points])
    )


def blocker() -> tuple[TrafficAircraft, ...]:
    point = ROUTE[0]
    return (TrafficAircraft(2, point.lat, point.lon, 10, 0, 0, 5, 5),)


def replacement(fence: FlightBoundary, traffic=None):
    return outward_replacement(
        position=HOME,
        original=ROUTE[0],
        route=ROUTE,
        route_index=0,
        traffic=blocker() if traffic is None else traffic,
        age_s=0.1,
        boundary=fence,
        avoider=ObstacleAvoider(),
    )


def test_blocked_point_moves_outward_within_twenty_metres() -> None:
    fence = boundary(50)
    candidate = replacement(fence)
    assert candidate is not None
    east, north = east_north_coordinate_offset_m(
        ROUTE[0].lat, ROUTE[0].lon, candidate.lat, candidate.lon
    )
    assert 0 < math.hypot(east, north) <= 20.01
    assert east > 0 and north > 0
    assert replacement_geometry_valid(ROUTE[0], candidate, ROUTE, 0, fence)


def test_tight_boundary_or_large_buffer_keeps_waypoint_blocked() -> None:
    assert replacement(boundary(20)) is None
    large = blocker()[0]
    assert (
        replacement(
            boundary(50),
            (
                TrafficAircraft(
                    large.aircraft_index,
                    large.lat,
                    large.lon,
                    large.altitude_agl_m,
                    large.speed_mps,
                    large.heading_deg,
                    25,
                    large.vertical_keep_away_m,
                ),
            ),
        )
        is None
    )


def test_replacement_only_moves_farther_out_on_new_snapshot() -> None:
    fence = boundary(50)
    first = replacement(fence)
    assert first is not None
    first_e, first_n = east_north_coordinate_offset_m(
        ROUTE[0].lat, ROUTE[0].lon, first.lat, first.lon
    )
    first_shift_m = math.hypot(first_e, first_n)
    later = outward_replacement(
        position=HOME,
        original=ROUTE[0],
        route=ROUTE,
        route_index=0,
        traffic=blocker(),
        age_s=0.2,
        boundary=fence,
        avoider=ObstacleAvoider(),
        min_shift_m=first_shift_m,
    )
    assert later is not None
    later_e, later_n = east_north_coordinate_offset_m(
        ROUTE[0].lat, ROUTE[0].lon, later.lat, later.lon
    )
    assert math.hypot(later_e, later_n) > first_shift_m + 0.5


def test_missing_route_or_stale_snapshot_cannot_relocate() -> None:
    fence = boundary(50)
    assert (
        outward_replacement(
            position=HOME,
            original=ROUTE[0],
            route=ROUTE[:2],
            route_index=0,
            traffic=blocker(),
            age_s=0.1,
            boundary=fence,
            avoider=ObstacleAvoider(),
        )
        is None
    )
    assert (
        outward_replacement(
            position=HOME,
            original=ROUTE[0],
            route=ROUTE,
            route_index=0,
            traffic=blocker(),
            age_s=2.0,
            boundary=fence,
            avoider=ObstacleAvoider(),
        )
        is None
    )
