import math

import pytest
from shapely.geometry import Point, Polygon

from orthomosaic.coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
    enu_to_coordinate,
    rotate_point,
)
from orthomosaic.decompose import HORIZONTAL, decompose_polygon, order_cells
from orthomosaic.footprint import derive_altitude, ground_footprint, spacings
from orthomosaic.models import CameraSpec, ScanRequest
from orthomosaic.pathing import _candidate_angles, _evaluate_orientation, plan_orthomosaic_path
from orthomosaic.sweep import SweepParams, default_turn_penalty_m, path_length, sweep_cell
from utils.src.types import Coordinate

ANCHOR = Coordinate(43.5, -80.5, 0.0)
ARDUCAM = CameraSpec(
    image_width_px=1280,
    image_height_px=720,
    horizontal_fov_rad=math.radians(80),
    vertical_fov_rad=math.radians(55),
)


def enu_rect_boundary(width_m, height_m, rot_deg=0.0, anchor=ANCHOR):
    corners = [(0.0, 0.0), (width_m, 0.0), (width_m, height_m), (0.0, height_m)]
    if rot_deg:
        corners = [rotate_point(c, math.radians(rot_deg)) for c in corners]
    return [enu_to_coordinate(anchor, east, north) for east, north in corners]


def make_request(boundary, gsd=0.03, **kwargs):
    return ScanRequest(boundary=boundary, camera=ARDUCAM, target_gsd_m=gsd, **kwargs)


def is_covered(polygon, points, swath, frame, angle_deg):
    """True if a grid of sample points inside ``polygon`` all fall in a footprint."""

    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    rects = []
    for x, y in points:
        half_frame = frame / 2.0
        half_swath = swath / 2.0
        corners = [
            (x - half_frame * cosine + half_swath * sine, y - half_frame * sine - half_swath * cosine),
            (x + half_frame * cosine + half_swath * sine, y + half_frame * sine - half_swath * cosine),
            (x + half_frame * cosine - half_swath * sine, y + half_frame * sine + half_swath * cosine),
            (x - half_frame * cosine - half_swath * sine, y - half_frame * sine + half_swath * cosine),
        ]
        rects.append(Polygon(corners))

    minx, miny, maxx, maxy = polygon.bounds
    step = min(swath, frame) / 4.0
    x = minx
    while x <= maxx:
        y = miny
        while y <= maxy:
            point = Point(x, y)
            if polygon.contains(point) and not any(rect.contains(point) for rect in rects):
                return False
            y += step
        x += step
    return True


def enu_of(waypoints, boundary):
    anchor = anchor_for_boundary(boundary)
    return [coordinate_to_enu(anchor, wp) for wp in waypoints]


def sweep_params():
    altitude = derive_altitude(ARDUCAM, 0.03)
    width, length = ground_footprint(altitude, ARDUCAM)
    cap, line = spacings(width, length, 0.75, 0.65)
    return SweepParams(swath_m=width, frame_m=length, line_spacing_m=line, capture_spacing_m=cap)


# --- angle generation ------------------------------------------------------


def test_candidate_angles_include_edge_angles_and_grid():
    polygon = Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)])
    angles = _candidate_angles(polygon, step_deg=5.0)
    assert 0.0 in angles
    assert 90.0 in angles
    assert 5.0 in angles
    assert 175.0 in angles
    assert len(angles) >= 36


def test_candidate_angles_are_mod_180():
    polygon = Polygon([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    angles = _candidate_angles(polygon, step_deg=5.0)
    assert all(0.0 <= angle < 180.0 for angle in angles)


def test_candidate_angles_rejects_nonpositive_step():
    polygon = Polygon([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    with pytest.raises(ValueError):
        _candidate_angles(polygon, step_deg=0.0)


# --- end-to-end planning ---------------------------------------------------


def test_plan_rectangle_prefers_long_axis():
    boundary = enu_rect_boundary(200.0, 50.0)
    plan = plan_orthomosaic_path(make_request(boundary))
    assert plan.sweep_angle_deg == pytest.approx(0.0, abs=1e-9)


def test_plan_rotated_rectangle_prefers_its_long_axis():
    boundary = enu_rect_boundary(200.0, 50.0, rot_deg=30.0)
    plan = plan_orthomosaic_path(make_request(boundary))
    assert plan.sweep_angle_deg == pytest.approx(30.0, abs=1e-6)


def test_selected_orientation_has_lowest_cost():
    boundary = enu_rect_boundary(200.0, 50.0, rot_deg=30.0)
    polygon = Polygon(boundary_to_enu(anchor_for_boundary(boundary), boundary))
    params = sweep_params()
    penalty = default_turn_penalty_m(params.line_spacing_m)

    plan = plan_orthomosaic_path(make_request(boundary))
    costs = {
        angle: _evaluate_orientation(polygon, math.radians(angle), params, penalty, None)[0]
        for angle in [0.0, 30.0, 60.0, 90.0]
    }
    assert costs[30.0] < costs[0.0]
    assert costs[30.0] < costs[60.0]
    assert costs[30.0] < costs[90.0]
    assert plan.sweep_angle_deg == pytest.approx(30.0, abs=1e-6)


def test_coverage_preserved_after_unrotation():
    boundary = enu_rect_boundary(200.0, 50.0)
    plan = plan_orthomosaic_path(make_request(boundary))
    polygon = Polygon(boundary_to_enu(anchor_for_boundary(boundary), boundary))
    assert is_covered(
        polygon,
        enu_of(plan.capture_waypoints, boundary),
        plan.footprint_width_m,
        plan.footprint_length_m,
        plan.sweep_angle_deg,
    )


def test_coverage_preserved_rotated_rectangle():
    boundary = enu_rect_boundary(200.0, 50.0, rot_deg=30.0)
    plan = plan_orthomosaic_path(make_request(boundary))
    polygon = Polygon(boundary_to_enu(anchor_for_boundary(boundary), boundary))
    assert is_covered(
        polygon,
        enu_of(plan.capture_waypoints, boundary),
        plan.footprint_width_m,
        plan.footprint_length_m,
        plan.sweep_angle_deg,
    )


def test_gps_output_consistent_with_boundary():
    boundary = enu_rect_boundary(200.0, 50.0)
    plan = plan_orthomosaic_path(make_request(boundary))
    assert len(plan.capture_waypoints) > 0
    assert plan.photo_count == len(plan.capture_waypoints)
    assert plan.altitude_agl_m == pytest.approx(derive_altitude(ARDUCAM, 0.03))

    for wp in plan.capture_waypoints + plan.path_waypoints:
        assert wp.alt == pytest.approx(plan.altitude_agl_m)
        assert -90.0 <= wp.lat <= 90.0
        assert -180.0 <= wp.lon <= 180.0

    assert set(plan.capture_waypoints).issubset(set(plan.path_waypoints))

    polygon = Polygon(boundary_to_enu(anchor_for_boundary(boundary), boundary))
    minx, miny, maxx, maxy = polygon.bounds
    xs = [p[0] for p in enu_of(plan.capture_waypoints, boundary)]
    ys = [p[1] for p in enu_of(plan.capture_waypoints, boundary)]
    assert min(xs) >= minx - 1e-6 and max(xs) <= maxx + 1e-6
    assert min(ys) >= miny - 1e-6 and max(ys) <= maxy + 1e-6


def test_achieved_gsd_meets_target():
    plan = plan_orthomosaic_path(make_request(enu_rect_boundary(200.0, 50.0)))
    assert plan.achieved_gsd_m <= 0.03 + 1e-12
    assert plan.achieved_gsd_m == pytest.approx(0.03, rel=1e-9)


def test_path_starts_at_start_when_provided():
    start = enu_rect_boundary(200.0, 50.0)[3]
    plan = plan_orthomosaic_path(make_request(enu_rect_boundary(200.0, 50.0), start=start))
    first_enu = coordinate_to_enu(ANCHOR, plan.path_waypoints[0])
    start_enu = coordinate_to_enu(ANCHOR, start)
    assert math.hypot(first_enu[0] - start_enu[0], first_enu[1] - start_enu[1]) < 1e-3


def test_plan_deterministic_without_start():
    boundary = enu_rect_boundary(200.0, 50.0)
    plan_a = plan_orthomosaic_path(make_request(boundary))
    plan_b = plan_orthomosaic_path(make_request(boundary))
    assert plan_a.capture_waypoints == plan_b.capture_waypoints
    assert plan_a.sweep_angle_deg == plan_b.sweep_angle_deg


def test_plan_rejects_short_boundary():
    boundary = [Coordinate(43.5, -80.5, 0.0), Coordinate(43.6, -80.4, 0.0)]
    with pytest.raises(ValueError, match="at least 3"):
        plan_orthomosaic_path(make_request(boundary))


def test_plan_rejects_degenerate_boundaries():
    degenerate = [
        [Coordinate(43.5, -80.5, 0.0), Coordinate(43.5, -80.5, 0.0), Coordinate(43.5, -80.5, 0.0)],
        [
            Coordinate(43.5, -80.5, 0.0),
            Coordinate(43.5009, -80.4991, 0.0),
            Coordinate(43.5, -80.4991, 0.0),
            Coordinate(43.5009, -80.5, 0.0),
        ],
    ]
    for boundary in degenerate:
        with pytest.raises(ValueError):
            plan_orthomosaic_path(make_request(boundary))


# --- assembly: no unnecessary pathing --------------------------------------


def test_assembly_is_exact_concatenation():
    u_shape = [
        (0.0, 0.0),
        (200.0, 0.0),
        (200.0, 200.0),
        (140.0, 200.0),
        (140.0, 100.0),
        (60.0, 100.0),
        (60.0, 200.0),
        (0.0, 200.0),
    ]
    polygon = Polygon(u_shape)
    params = sweep_params()
    penalty = default_turn_penalty_m(params.line_spacing_m)

    cost, capture, path, turns, plan_length, cell_count = _evaluate_orientation(
        polygon, 0.0, params, penalty, None
    )
    assert cell_count == 3

    cells = order_cells(decompose_polygon(polygon, HORIZONTAL), None)
    per_cell = [sweep_cell(cell, params) for cell in cells]

    expected_straight = sum(result.straight_length_m for result in per_cell)
    expected_turns = sum(result.turn_count for result in per_cell) + (len(cells) - 1)
    assert cost == pytest.approx(expected_straight + penalty * expected_turns)
    assert turns == expected_turns

    assembled = []
    for result in per_cell:
        assembled.extend(result.path_waypoints)
    assert path == assembled
    assert plan_length == pytest.approx(path_length(assembled))
    assert len(capture) == sum(len(result.capture_waypoints) for result in per_cell)