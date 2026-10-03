import math

import pytest
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from orthomosaic.decompose import HORIZONTAL, VERTICAL, Cell
from orthomosaic.sweep import (
    SweepParams,
    SweepResult,
    _dedupe,
    coverage_cost,
    default_turn_penalty_m,
    path_length,
    sweep_cell,
)

RECT_100x50 = Polygon([(0, 0), (100, 0), (100, 50), (0, 50)])
RECT_100x10 = Polygon([(0, 0), (100, 0), (100, 10), (0, 10)])
RECT_8x40 = Polygon([(0, 0), (8, 0), (8, 40), (0, 40)])
TRIANGLE = Polygon([(0, 0), (100, 0), (100, 50)])
TAPER = Polygon([(0, 0), (100, 0), (50, 100)])


def params(swath=10.0, frame=20.0, side=0.5, forward=0.5) -> SweepParams:
    return SweepParams(
        swath_m=swath,
        frame_m=frame,
        line_spacing_m=swath * (1.0 - side),
        capture_spacing_m=frame * (1.0 - forward),
    )


def footprints(points, swath, frame):
    return unary_union(
        [box(x - frame / 2.0, y - swath / 2.0, x + frame / 2.0, y + swath / 2.0) for x, y in points]
    )


def test_rectangle_predictable_passes():
    result = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params())
    assert result.turn_count == 8  # 9 passes
    assert result.straight_length_m == pytest.approx(900.0)  # 9 x 100 m
    assert len(result.capture_waypoints) == 81  # 9 passes x 9 captures
    assert result.capture_waypoints[0] == (10.0, 5.0)
    assert result.capture_waypoints[-1] == (90.0, 45.0)
    assert result.path_waypoints[0] == (0.0, 5.0)
    assert result.path_waypoints[10] == (100.0, 5.0)
    assert result.path_waypoints[11] == (100.0, 10.0)  # transit to pass 2
    assert result.path_length_m == pytest.approx(940.0)  # 900 + 8 transits x 5 m


def test_different_overlaps_change_counts():
    zero_side = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params(side=0.0))
    assert zero_side.turn_count == 4  # 5 passes

    zero_forward = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params(forward=0.0))
    assert len(zero_forward.capture_waypoints) == 45  # 9 passes x 5 captures

    half = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params())
    assert half.turn_count == 8
    assert len(half.capture_waypoints) == 81


def test_cell_smaller_than_footprint():
    small = Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])
    result = sweep_cell(Cell(small, HORIZONTAL), params())
    assert result.turn_count == 0
    assert len(result.capture_waypoints) == 1
    assert result.capture_waypoints[0] == (2.5, 5.0)  # single pass at ymin + swath/2
    assert result.path_waypoints == [(0.0, 5.0), (2.5, 5.0), (5.0, 5.0)]
    assert result.straight_length_m == pytest.approx(5.0)


def test_pass_shorter_than_capture_spacing():
    result = sweep_cell(Cell(RECT_8x40, HORIZONTAL), params(frame=6.0, forward=0.0))
    xs = sorted(x for x, _ in result.capture_waypoints)
    assert all(0.0 - 1e-9 <= x <= 8.0 + 1e-9 for x in xs)
    assert len(result.capture_waypoints) == 14  # 7 passes x 2 captures
    pass_zero = [x for x, y in result.capture_waypoints if y == pytest.approx(5.0)]
    assert sorted(pass_zero) == [3.0, 5.0]


@pytest.mark.parametrize(
    "polygon",
    [RECT_100x50, TRIANGLE, RECT_8x40, TAPER],
)
def test_boundary_coverage_is_complete(polygon):
    result = sweep_cell(Cell(polygon, HORIZONTAL), params())
    union = footprints(result.capture_waypoints, 10.0, 20.0)
    assert polygon.difference(union).area < 1e-6


def test_coverage_complete_vertical_axis():
    result = sweep_cell(Cell(RECT_100x50, VERTICAL), params())
    assert result.turn_count == 18  # 19 passes along y
    assert result.straight_length_m == pytest.approx(950.0)  # 19 x 50 m
    assert len(result.capture_waypoints) == 76  # 19 x 4 captures
    union = footprints(result.capture_waypoints, 20.0, 10.0)  # swath on x, frame on y
    assert RECT_100x50.difference(union).area < 1e-6


def test_turn_and_path_length():
    result = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params())
    assert result.straight_length_m == pytest.approx(900.0)
    assert result.turn_count == 8
    assert result.path_length_m == pytest.approx(940.0)
    assert path_length(result.path_waypoints) == pytest.approx(result.path_length_m)


def test_orientation_cost_prefers_long_axis():
    penalty = default_turn_penalty_m(5.0)  # pi * line_spacing
    horizontal = sweep_cell(Cell(RECT_100x10, HORIZONTAL), params())
    vertical = sweep_cell(Cell(RECT_100x10, VERTICAL), params())

    assert horizontal.turn_count == 0
    assert vertical.turn_count == 18
    assert horizontal.straight_length_m == pytest.approx(100.0)
    assert vertical.straight_length_m == pytest.approx(190.0)

    cost_h = coverage_cost(horizontal, penalty)
    cost_v = coverage_cost(vertical, penalty)
    assert cost_h < cost_v
    assert cost_v - cost_h == pytest.approx(90.0 + 18.0 * penalty)

    cost_h_zero = coverage_cost(horizontal, 0.0)
    cost_v_zero = coverage_cost(vertical, 0.0)
    assert cost_h_zero < cost_v_zero


def test_default_turn_penalty_value():
    assert default_turn_penalty_m(5.0) == pytest.approx(math.pi * 5.0)


def test_dedupe_removes_consecutive_duplicates():
    points = [(0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (1.0, 1.0), (1.0, 1.0), (2.0, 2.0)]
    assert _dedupe(points) == [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]


def test_dedupe_keeps_distinct_points():
    points = [(0.0, 0.0), (1e-3, 1e-3), (2.0, 0.0)]
    assert _dedupe(points) == points


def test_no_near_duplicate_path_points_on_tapering_shape():
    result = sweep_cell(Cell(TAPER, HORIZONTAL), params(swath=4.0, frame=10.0, side=0.5, forward=0.5))
    assert len(result.path_waypoints) >= 2
    for a, b in zip(result.path_waypoints, result.path_waypoints[1:]):
        assert math.hypot(a[0] - b[0], a[1] - b[1]) > 1e-6


def test_sweep_rejects_invalid_params():
    bad = SweepParams(swath_m=10.0, frame_m=20.0, line_spacing_m=15.0, capture_spacing_m=5.0)
    with pytest.raises(ValueError):
        sweep_cell(Cell(RECT_100x50, HORIZONTAL), bad)

    bad2 = SweepParams(swath_m=0.0, frame_m=20.0, line_spacing_m=5.0, capture_spacing_m=5.0)
    with pytest.raises(ValueError):
        sweep_cell(Cell(RECT_100x50, HORIZONTAL), bad2)


def test_sweep_result_is_frozen():
    result = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params())
    with pytest.raises(Exception):
        result.straight_length_m = 0.0  # frozen dataclass


def test_sweep_result_type():
    result = sweep_cell(Cell(RECT_100x50, HORIZONTAL), params())
    assert isinstance(result, SweepResult)