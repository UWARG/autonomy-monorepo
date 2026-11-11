import math
import random

import pytest

from deconfliction import closest_points, find_conflicts, orientation, to_segments, find_conflicts_sweep
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate

ORIGIN = Coordinate(43.4717, -80.5414, 0.0)


def test_orientation_left():
    assert orientation((0, 0), (10, 10), (0, 10)) > 0


def test_orientation_right():
    assert orientation((0, 0), (-10, 10), (0, 10)) < 0


def test_orientation_collinear():
    assert orientation((0, 0), (10, 10), (5, 5)) == 0


def at(east_m, north_m):
    """A Coordinate this many meters east and north of ORIGIN."""
    return enu_to_coordinate(ORIGIN, east_m, north_m)


@pytest.mark.parametrize(
    ("p1", "p2", "q1", "q2", "expected"),
    [
        ((0, 0), (10, 10), (0, 10), (10, 0), 0),        # X crossing
        ((0, 0), (10, 0), (0, 5), (10, 5), 5),          # parallel, 5 m apart
        ((0, 0), (10, 0), (5, 0), (5, 10), 0),          # T touch
        ((10, 0), (10, 20), (10, 5), (10, 30), 0),      # overlap on shared boundary
        ((0, 0), (5, 0), (7, 0), (10, 0), 2),           # same line, 2 m gap
        ((3, 4), (3, 4), (0, 0), (10, 0), 4),           # hovering drone, 4 m away
    ],
    ids=["x", "parallel", "t", "overlap", "collinear_gap", "hovering"],
)
def test_closest_distance(p1, p2, q1, q2, expected):
    assert math.dist(*closest_points(p1, p2, q1, q2)) == pytest.approx(expected)
    assert math.dist(*closest_points(q1, q2, p1, p2)) == pytest.approx(expected)


def test_closest_points_at_crossing():
    point_a, point_b = closest_points((0, 0), (10, 10), (0, 10), (10, 0))
    assert point_a == pytest.approx((5, 5))
    assert point_b == pytest.approx((5, 5))


def test_to_segments():
    assert to_segments([(0, 0), (10, 10), (20, 10)]) == [
        ((0, 0), (10, 10)),
        ((10, 10), (20, 10)),
    ]
    assert to_segments([(0, 0)]) == [((0, 0), (0, 0))]


def test_find_conflicts_crossing():
    paths = {"A": [at(0, 0), at(100, 100)], "B": [at(0, 100), at(100, 0)]}
    [conflict] = find_conflicts(paths, min_separation_m=5)
    assert (conflict.drone_a, conflict.segment_a) == ("A", 0)
    assert (conflict.drone_b, conflict.segment_b) == ("B", 0)
    assert conflict.distance_m == pytest.approx(0, abs=0.05)
    assert conflict.point_a.lat == pytest.approx(at(50, 50).lat, abs=1e-6)
    assert conflict.point_a.lon == pytest.approx(at(50, 50).lon, abs=1e-6)


def test_find_conflicts_near_miss():
    paths = {"A": [at(0, 0), at(100, 0)], "B": [at(0, 3), at(100, 3)]}
    [conflict] = find_conflicts(paths, min_separation_m=5)
    assert conflict.distance_m == pytest.approx(3, abs=0.05)


def test_find_conflicts_clear_of_separation():
    paths = {"A": [at(0, 0), at(100, 0)], "B": [at(0, 10), at(100, 10)]}
    assert find_conflicts(paths, min_separation_m=5) == []


def test_find_conflicts_ignores_a_drones_own_legs():
    paths = {"A": [at(0, 0), at(50, 50), at(100, 0)], "B": [at(0, 200), at(100, 200)]}
    assert find_conflicts(paths, min_separation_m=5) == []


def test_find_conflicts_hovering_drone():
    paths = {"A": [at(50, 2)], "B": [at(0, 0), at(100, 0)]}
    [conflict] = find_conflicts(paths, min_separation_m=5)
    assert conflict.segment_a == 0
    assert conflict.distance_m == pytest.approx(2, abs=0.05)


def test_find_conflicts_rejects_bad_separation():
    with pytest.raises(ValueError):
        find_conflicts({}, min_separation_m=0)


def conflict_pairs(conflicts):
    """Which segment pairs conflict, ignoring order."""
    return sorted(
        tuple(sorted([(c.drone_a, c.segment_a), (c.drone_b, c.segment_b)]))
        for c in conflicts
    )


def test_sweep_matches_brute_force():
    rng = random.Random(0)
    for _ in range(200):
        paths = {
            f"D{d}": [
                at(rng.uniform(0, 200), rng.uniform(0, 200))
                for _ in range(rng.randint(1, 5))
            ]
            for d in range(rng.randint(2, 6))
        }
        assert conflict_pairs(find_conflicts_sweep(paths, 10)) == conflict_pairs(
            find_conflicts(paths, 10)
        )


def test_sweep_skips_far_apart_drones():
    paths = {
        "A": [at(0, 0), at(10, 0)],
        "B": [at(3, 2), at(8, 2)],
        "C": [at(100, 0), at(110, 0)],
    }
    [conflict] = find_conflicts_sweep(paths, min_separation_m=5)
    assert {conflict.drone_a, conflict.drone_b} == {"A", "B"}
    assert conflict.distance_m == pytest.approx(2, abs=0.05)
    