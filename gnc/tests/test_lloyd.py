import math

import pytest
from shapely.geometry import Point, Polygon

from lloyd import (
    coverage_cost,
    coverage_cost_local,
    lloyd_relax,
    lloyd_step,
    step_local,
    voronoi_cells,
)
from lloyd.relaxation import _squared_distance_integral
from utils.src.coordinates import coordinate_to_enu, enu_to_coordinate
from utils.src.types import Coordinate

ANCHOR = Coordinate(43.5, -80.5, 0.0)
SQUARE = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
# L-shape: a 100 m square with the top-right 50 m quarter removed.
L_SHAPE = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (50.0, 50.0), (50.0, 100.0), (0.0, 100.0)]


def gps_boundary(corners):
    return [enu_to_coordinate(ANCHOR, east, north) for east, north in corners]


def gps_positions(points, alt=0.0):
    return {
        drone_id: enu_to_coordinate(ANCHOR, east, north, alt)
        for drone_id, (east, north) in points.items()
    }


def local(positions):
    return {i: coordinate_to_enu(ANCHOR, p) for i, p in positions.items()}


def assert_near(actual, expected, tol_m):
    assert math.dist(actual, expected) < tol_m, (actual, expected)


def test_single_drone_moves_to_region_centroid():
    moved = lloyd_step(gps_positions({7: (10.0, 10.0)}), gps_boundary(SQUARE))

    assert set(moved) == {7}
    assert_near(local(moved)[7], (50.0, 50.0), 0.01)


def test_two_drones_split_a_rectangle_in_half():
    rectangle = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)]
    start = gps_positions({1: (10.0, 20.0), 2: (30.0, 30.0)})

    result = lloyd_relax(start, gps_boundary(rectangle), tolerance_m=0.001)

    assert result.converged
    end = local(result.positions)
    assert_near(end[1], (25.0, 25.0), 0.05)
    assert_near(end[2], (75.0, 25.0), 0.05)


def test_four_drones_settle_on_quadrant_centres():
    start = gps_positions({1: (40.0, 45.0), 2: (55.0, 40.0), 3: (60.0, 60.0), 4: (45.0, 55.0)})

    result = lloyd_relax(start, gps_boundary(SQUARE), tolerance_m=0.001, max_iterations=200)

    assert result.converged
    end = sorted(local(result.positions).values())
    for actual, expected in zip(end, [(25.0, 25.0), (25.0, 75.0), (75.0, 25.0), (75.0, 75.0)]):
        assert_near(actual, expected, 0.1)


def test_ids_and_altitudes_are_kept():
    start = {
        12: enu_to_coordinate(ANCHOR, 20.0, 20.0, 30.0),
        5: enu_to_coordinate(ANCHOR, 70.0, 60.0, 45.0),
    }

    moved = lloyd_step(start, gps_boundary(SQUARE))

    assert set(moved) == {12, 5}
    assert moved[12].alt == 30.0
    assert moved[5].alt == 45.0


def test_drones_stay_inside_a_concave_region():
    region = Polygon(L_SHAPE)
    # Clustered near the inner corner so cells wrap around the notch.
    points = {i: (45.0 + i, 45.0 + 0.5 * i) for i in range(5)}

    for _ in range(30):
        points = step_local(points, region)
        for point in points.values():
            assert region.buffer(1e-6).covers(Point(point))


def test_centroid_outside_a_concave_cell_is_pulled_inside_the_cell():
    # One drone over an L-shape: the L's centroid lies inside it, so use a
    # thin U-shape whose centroid sits in the empty middle.
    u_shape = Polygon([(0, 0), (30, 0), (30, 30), (20, 30), (20, 10), (10, 10), (10, 30), (0, 30)])
    assert not u_shape.covers(u_shape.centroid)

    moved = step_local({1: (5.0, 5.0)}, u_shape)

    assert u_shape.buffer(1e-6).covers(Point(moved[1]))


def test_drone_outside_the_region_is_brought_back():
    region = Polygon(SQUARE)
    points = {1: (50.0, 50.0), 2: (500.0, 500.0)}

    moved = step_local(points, region)

    assert_near(moved[2], (100.0, 100.0), 1e-6)


def test_two_drones_outside_the_same_corner_land_apart():
    region = Polygon(SQUARE)
    points = {1: (50.0, 50.0), 2: (500.0, 500.0), 3: (600.0, 600.0)}

    moved = step_local(points, region)

    # Far enough apart that they can't collide, not just distinct points.
    assert math.dist(moved[2], moved[3]) >= 2.0 - 1e-6
    for point in moved.values():
        assert region.buffer(1e-6).covers(Point(point))


def test_stranded_drone_in_a_crowded_region_still_lands_inside():
    # Every point of a 1 m square is within the 2 m clearance of drone 1.
    region = Polygon([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    points = {1: (0.5, 0.5), 2: (50.0, 50.0)}

    moved = step_local(points, region)

    assert region.buffer(1e-6).covers(Point(moved[2]))
    assert math.dist(moved[1], moved[2]) > 1e-3


def test_relax_recovers_drones_that_start_outside():
    start = gps_positions({1: (50.0, 50.0), 2: (500.0, 500.0), 3: (600.0, 600.0)})

    result = lloyd_relax(start, gps_boundary(SQUARE), max_iterations=200)

    assert result.converged
    for point in local(result.positions).values():
        assert Polygon(SQUARE).buffer(0.01).covers(Point(point))


def test_cells_partition_the_region():
    region = Polygon(L_SHAPE)
    points = {1: (10.0, 10.0), 2: (80.0, 20.0), 3: (20.0, 80.0), 4: (40.0, 40.0)}

    cells = voronoi_cells(points, region)

    assert sum(cell.area for cell in cells.values()) == pytest.approx(region.area)
    for drone_id, point in points.items():
        assert cells[drone_id].covers(Point(point))


def test_collinear_drones_are_handled():
    region = Polygon(SQUARE)
    points = {1: (20.0, 50.0), 2: (50.0, 50.0), 3: (80.0, 50.0)}

    cells = voronoi_cells(points, region)

    assert sum(cell.area for cell in cells.values()) == pytest.approx(region.area)


def test_coverage_cost_never_increases():
    start = gps_positions({i: (5.0 + 3.0 * i, 5.0 + 2.0 * i) for i in range(6)})
    boundary = gps_boundary(L_SHAPE)

    result = lloyd_relax(start, boundary, max_iterations=40)
    costs = [coverage_cost(step, boundary) for step in result.history]

    assert costs[-1] < 0.5 * costs[0]
    for before, after in zip(costs, costs[1:]):
        assert after <= before + 1e-6


def test_squared_distance_integral_matches_closed_form():
    unit = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])

    # Polar moment of a unit square: 1/6 about its centre, 2/3 about a corner.
    assert _squared_distance_integral(unit, (0.5, 0.5)) == pytest.approx(1 / 6)
    assert _squared_distance_integral(unit, (0.0, 0.0)) == pytest.approx(2 / 3)


def test_squared_distance_integral_sums_the_parts_of_a_split_cell():
    left = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
    right = Polygon([(3, 0), (4, 0), (4, 1), (3, 1)])
    split = left.union(right)
    assert split.geom_type == "MultiPolygon"

    # About (2, 0.5) each unit square is its centre's 1/6 plus 1.5^2 = 2.25.
    assert _squared_distance_integral(split, (2.0, 0.5)) == pytest.approx(2 * (1 / 6 + 2.25))


def test_coverage_cost_of_one_drone_at_square_centre():
    # Mean of x^2 + y^2 over a 100 m square about its centre is 100^2 / 6.
    cost = coverage_cost_local({1: (50.0, 50.0)}, Polygon(SQUARE))

    assert cost == pytest.approx(100.0**2 / 6)


def test_history_has_one_entry_per_step_plus_start():
    start = gps_positions({1: (10.0, 10.0), 2: (90.0, 90.0)})

    result = lloyd_relax(start, gps_boundary(SQUARE), max_iterations=3, tolerance_m=1e-9)

    assert not result.converged
    assert result.iterations == 3
    assert len(result.history) == 4
    assert result.history[0] == start
    assert result.history[-1] == result.positions


def test_relax_stops_once_drones_stop_moving():
    already_even = gps_positions({1: (25.0, 50.0), 2: (75.0, 50.0)})

    result = lloyd_relax(already_even, gps_boundary([(0, 0), (100, 0), (100, 100), (0, 100)]))

    assert result.converged
    assert result.iterations == 1


@pytest.mark.parametrize(
    "positions",
    [
        {},
        {1: (10.0, 10.0), 2: (10.0, 10.0)},
        {1: (math.nan, 10.0)},
    ],
)
def test_bad_positions_are_rejected(positions):
    with pytest.raises(ValueError):
        step_local(positions, Polygon(SQUARE))


def test_non_finite_gps_position_is_rejected():
    with pytest.raises(ValueError):
        lloyd_step({1: Coordinate(math.nan, -80.5, 0.0)}, gps_boundary(SQUARE))


@pytest.mark.parametrize(
    "corners",
    [
        [(0.0, 0.0), (10.0, 0.0)],
        [(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)],  # bow-tie
        [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)],  # zero area
    ],
)
def test_bad_boundaries_are_rejected(corners):
    with pytest.raises(ValueError):
        lloyd_step(gps_positions({1: (1.0, 1.0)}), gps_boundary(corners))


@pytest.mark.parametrize("kwargs", [{"max_iterations": 0}, {"tolerance_m": 0.0}, {"tolerance_m": math.nan}])
def test_bad_relax_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        lloyd_relax(gps_positions({1: (1.0, 1.0)}), gps_boundary(SQUARE), **kwargs)
