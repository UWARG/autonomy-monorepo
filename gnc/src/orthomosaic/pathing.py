"""Top-level orthomosaic scan-plan orchestration."""

from __future__ import annotations

import math

from shapely.geometry import Polygon

from .coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
    enu_to_coordinate,
    rotate_point,
    rotate_points,
    rotate_polygon,
)
from .decompose import HORIZONTAL, decompose_polygon, order_cells, validate_polygon
from .footprint import (
    derive_altitude,
    ground_footprint,
    ground_sample_distance,
    spacings,
)
from .models import ScanPlan, ScanRequest
from .sweep import (
    SweepParams,
    SweepResult,
    default_turn_penalty_m,
    path_length,
    sweep_cell,
)

_Evaluation = tuple[
    float,
    list[tuple[float, float]],
    list[tuple[float, float]],
    int,
    float,
    int,
]


def plan_orthomosaic_path(request: ScanRequest) -> ScanPlan:
    """Plans an orthomosaic lawn-mower sweep for ``request.boundary``.

    The sweep orientation is chosen by evaluating candidate angles and
    keeping the one with the lowest coverage cost.
    """

    if len(request.boundary) < 3:
        raise ValueError("boundary must have at least 3 points")

    anchor = anchor_for_boundary(request.boundary)
    polygon = Polygon(boundary_to_enu(anchor, request.boundary))
    validate_polygon(polygon)

    altitude = derive_altitude(request.camera, request.target_gsd_m)
    width, length = ground_footprint(altitude, request.camera)
    gsd_x, gsd_y = ground_sample_distance(altitude, request.camera)
    capture_spacing, line_spacing = spacings(
        width, length, request.forward_overlap, request.side_overlap
    )
    params = SweepParams(
        swath_m=width,
        frame_m=length,
        line_spacing_m=line_spacing,
        capture_spacing_m=capture_spacing,
    )
    turn_penalty = (
        request.turn_penalty_m
        if request.turn_penalty_m is not None
        else default_turn_penalty_m(line_spacing)
    )

    start_point = (
        coordinate_to_enu(anchor, request.start) if request.start is not None else None
    )

    best_angle_deg: float | None = None
    best: _Evaluation | None = None
    for angle_deg in _candidate_angles(polygon, request.angle_step_deg):
        result = _evaluate_orientation(
            polygon, math.radians(angle_deg), params, turn_penalty, start_point
        )
        if best is None or result[0] < best[0]:
            best = result
            best_angle_deg = angle_deg

    assert best is not None
    _, capture_swept, path_swept, turns, plan_length, cell_count = best

    capture_enu = rotate_points(capture_swept, math.radians(best_angle_deg))
    path_enu = rotate_points(path_swept, math.radians(best_angle_deg))

    capture_waypoints = [
        enu_to_coordinate(anchor, east, north, altitude) for east, north in capture_enu
    ]
    path_waypoints = [
        enu_to_coordinate(anchor, east, north, altitude) for east, north in path_enu
    ]

    return ScanPlan(
        capture_waypoints=capture_waypoints,
        path_waypoints=path_waypoints,
        altitude_agl_m=altitude,
        achieved_gsd_m=max(gsd_x, gsd_y),
        footprint_width_m=width,
        footprint_length_m=length,
        line_spacing_m=line_spacing,
        capture_spacing_m=capture_spacing,
        sweep_angle_deg=best_angle_deg,
        cell_count=cell_count,
        estimated_path_length_m=plan_length,
        estimated_turn_count=turns,
        photo_count=len(capture_waypoints),
    )


def _candidate_angles(polygon: Polygon, step_deg: float) -> list[float]:
    """Candidate sweep angles: polygon edge directions plus a uniform grid."""

    if step_deg <= 0.0:
        raise ValueError("angle_step_deg must be positive")

    angles = set()
    coordinates = list(polygon.exterior.coords)[:-1]
    for index in range(len(coordinates)):
        start = coordinates[index]
        end = coordinates[(index + 1) % len(coordinates)]
        angle = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0])) % 180.0
        angles.add(angle)

    grid_count = max(1, int(180.0 / step_deg))
    angles.update((index * step_deg) % 180.0 for index in range(grid_count))
    return sorted(angles)


def _evaluate_orientation(
    polygon: Polygon,
    angle_rad: float,
    params: SweepParams,
    turn_penalty_m: float,
    start_point: tuple[float, float] | None,
) -> _Evaluation:
    """Sweeps ``polygon`` at one orientation and returns its cost and waypoints.

    Waypoints are in the sweep frame; the caller rotates them back to ENU.
    Returns ``(cost, capture, path, turn_count, path_length, cell_count)``.
    """

    swept = rotate_polygon(polygon, -angle_rad)
    cells = decompose_polygon(swept, HORIZONTAL)
    start_swept = rotate_point(start_point, -angle_rad) if start_point else None
    ordered = order_cells(cells, start_swept)

    capture_waypoints: list[tuple[float, float]] = []
    path_waypoints: list[tuple[float, float]] = []
    straight_length = 0.0
    turn_count = 0

    for cell in ordered:
        result: SweepResult = sweep_cell(cell, params)
        capture_waypoints.extend(result.capture_waypoints)
        path_waypoints.extend(result.path_waypoints)
        straight_length += result.straight_length_m
        turn_count += result.turn_count

    turn_count += max(0, len(ordered) - 1)

    if start_swept is not None and path_waypoints:
        path_waypoints = [start_swept] + path_waypoints

    cost = straight_length + turn_penalty_m * turn_count
    return (
        cost,
        capture_waypoints,
        path_waypoints,
        turn_count,
        path_length(path_waypoints),
        len(ordered),
    )