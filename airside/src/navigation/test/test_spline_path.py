"""Tests for the Catmull-Rom path and pure-pursuit aiming."""

import math

import pytest

from navigation.spline_path import SplinePath, aim_point

# An L-shaped route with uneven legs: 30 m north, then 10 m east.
ROUTE = [(0.0, 0.0, 10.0), (0.0, 30.0, 10.0), (10.0, 30.0, 10.0)]


def test_spline_passes_through_every_waypoint() -> None:
    path = SplinePath(ROUTE)

    for index, waypoint in enumerate(ROUTE):
        assert path.point_at(path.arc_at_waypoint(index)) == pytest.approx(waypoint)


def test_centripetal_spline_stays_close_to_the_legs() -> None:
    path = SplinePath(ROUTE)

    # Passing smoothly through the corner bulges slightly beyond it, but the
    # short leg does not make the long one swing wide or loop.
    assert max(point[0] for point in path.points) <= 10.0 + 1e-9
    assert max(point[1] for point in path.points) <= 30.0 + 1.5
    assert min(point[0] for point in path.points) >= -2.0


def test_aim_is_lookahead_ahead_along_the_path() -> None:
    aim = aim_point(SplinePath(ROUTE), 1, (0.0, 10.0, 10.0), lookahead_m=8.0, corner_lookahead_m=3.0)

    assert abs(aim[0]) < 2.0
    assert aim[1] == pytest.approx(18.0, abs=0.5)


def test_closing_on_a_waypoint_aims_at_it_instead_of_cutting() -> None:
    aim = aim_point(SplinePath(ROUTE), 1, (0.0, 25.0, 10.0), lookahead_m=8.0, corner_lookahead_m=3.0)

    assert aim == pytest.approx(ROUTE[1], abs=0.1)


def test_within_corner_lookahead_aims_past_the_waypoint() -> None:
    aim = aim_point(SplinePath(ROUTE), 1, (0.0, 29.0, 10.0), lookahead_m=8.0, corner_lookahead_m=3.0)

    # 3 m along the path from the drone: 2 m past the corner, onto the next leg.
    assert aim[0] > 1.0
    assert math.dist(aim, ROUTE[1]) < 2.5


def test_aim_stops_at_end_of_path() -> None:
    assert aim_point(SplinePath(ROUTE), 2, (9.0, 30.0, 10.0), lookahead_m=8.0, corner_lookahead_m=3.0) == ROUTE[-1]


def test_flying_in_to_the_first_waypoint_aims_straight_at_it() -> None:
    assert aim_point(SplinePath(ROUTE), 0, (0.0, -20.0, 10.0), lookahead_m=8.0, corner_lookahead_m=3.0) == ROUTE[0]


def test_close_to_the_first_waypoint_aims_past_it_along_the_path() -> None:
    aim = aim_point(SplinePath(ROUTE), 0, (0.0, -1.0, 10.0), lookahead_m=8.0, corner_lookahead_m=3.0)

    assert aim[1] == pytest.approx(2.0, abs=0.3)


def test_duplicate_waypoints_are_merged() -> None:
    path = SplinePath([ROUTE[0], ROUTE[0], ROUTE[1]])

    assert path.arc_at_waypoint(1) == 0.0
    assert path.arc_m[-1] == pytest.approx(30.0, abs=0.1)
    assert all(math.isfinite(c) for point in path.points for c in point)
