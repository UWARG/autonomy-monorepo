"""The companion boundary never treats an unapproved polygon as usable."""

import json

import pytest
from obstacle_avoidance.boundary import FlightBoundary
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate

HOME = Coordinate(43.0, -80.0, 10.0)


def polygon(points):
    return json.dumps(
        [
            [point.lat, point.lon]
            for point in (
                enu_to_coordinate(HOME, east, north) for east, north in points
            )
        ]
    )


def test_segment_must_stay_inside_inward_margin() -> None:
    boundary = FlightBoundary.from_json(
        polygon(((-30, -30), (30, -30), (30, 30), (-30, 30)))
    )
    center = HOME
    safe = enu_to_coordinate(HOME, 20, 0)
    edge = enu_to_coordinate(HOME, 28, 0)

    assert boundary.allows_segment(center, safe)
    assert not boundary.allows_point(edge)
    assert not boundary.allows_segment(center, edge)


def test_concave_boundary_rejects_segment_that_exits_polygon() -> None:
    boundary = FlightBoundary.from_json(
        polygon(
            (
                (-40, -40),
                (40, -40),
                (40, 40),
                (15, 40),
                (15, -10),
                (-15, -10),
                (-15, 40),
                (-40, 40),
            )
        )
    )
    left = enu_to_coordinate(HOME, -25, 20)
    right = enu_to_coordinate(HOME, 25, 20)

    assert boundary.allows_point(left)
    assert boundary.allows_point(right)
    assert not boundary.allows_segment(left, right)


@pytest.mark.parametrize(
    "raw", ["", "[]", "not-json", "[[91,0],[0,0],[0,1]]", "[[0,0],[1,1],[0,1],[1,0]]"]
)
def test_invalid_boundary_fails_closed(raw: str) -> None:
    with pytest.raises(ValueError, match="INVALID_FLIGHT_BOUNDARY"):
        FlightBoundary.from_json(raw)


def test_invalid_live_position_is_not_inside_boundary() -> None:
    boundary = FlightBoundary.from_json(
        polygon(((-30, -30), (30, -30), (30, 30), (-30, 30)))
    )
    invalid = Coordinate(float("nan"), HOME.lon, HOME.alt)

    assert not boundary.allows_point(invalid)
    assert not boundary.allows_segment(HOME, invalid)
