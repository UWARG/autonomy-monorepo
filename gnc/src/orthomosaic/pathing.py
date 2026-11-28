"""Top-level orthomosaic scan-plan orchestration."""

from __future__ import annotations

import math
from dataclasses import dataclass

from shapely.geometry import Polygon

from utils.src.coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
    enu_to_coordinate,
    rotate_point,
    rotate_points,
    rotate_polygon,
)
from utils.src.types import Coordinate

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

@dataclass(frozen=True)
class _SurveySweep:
    anchor: Coordinate
    altitude_m: float
    achieved_gsd_m: float
    params: SweepParams
    angle_deg: float
    cells: list[SweepResult]
    start_swept: tuple[float, float] | None

    def to_coordinates(self, points: list[tuple[float, float]]) -> list[Coordinate]:
        return [
            enu_to_coordinate(self.anchor, east, north, self.altitude_m)
            for east, north in rotate_points(points, math.radians(self.angle_deg))
        ]


def plan_orthomosaic_path(request: ScanRequest) -> ScanPlan:
    """Plans an orthomosaic lawn-mower sweep for ``request.boundary``.

    The sweep orientation is chosen by evaluating candidate angles and
    keeping the one with the lowest coverage cost.
    """

    survey = _plan_sweep(request)
    capture = [point for cell in survey.cells for point in cell.capture_waypoints]
    path = [point for cell in survey.cells for point in cell.path_waypoints]
    if survey.start_swept is not None and path:
        path.insert(0, survey.start_swept)

    return ScanPlan(
        capture_waypoints=survey.to_coordinates(capture),
        path_waypoints=survey.to_coordinates(path),
        altitude_agl_m=survey.altitude_m,
        achieved_gsd_m=survey.achieved_gsd_m,
        footprint_width_m=survey.params.swath_m,
        footprint_length_m=survey.params.frame_m,
        line_spacing_m=survey.params.line_spacing_m,
        capture_spacing_m=survey.params.capture_spacing_m,
        sweep_angle_deg=survey.angle_deg,
        cell_count=len(survey.cells),
        estimated_path_length_m=path_length(path),
        estimated_turn_count=_count_turns(survey.cells),
        photo_count=len(capture),
    )


def _plan_sweep(request: ScanRequest) -> _SurveySweep:
    """Choose an orientation and retain its ordered cell sweeps."""

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

    best_angle = 0.0
    best_cost: float | None = None
    best_cells: list[SweepResult] = []
    for angle_deg in _candidate_angles(polygon, request.angle_step_deg):
        cost, cells = _evaluate_orientation(
            polygon, math.radians(angle_deg), params, turn_penalty, start_point
        )
        if best_cost is None or cost < best_cost:
            best_angle, best_cost, best_cells = angle_deg, cost, cells

    return _SurveySweep(
        anchor=anchor,
        altitude_m=altitude,
        achieved_gsd_m=max(gsd_x, gsd_y),
        params=params,
        angle_deg=best_angle,
        cells=best_cells,
        start_swept=(
            rotate_point(start_point, -math.radians(best_angle)) if start_point is not None else None
        ),
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
) -> tuple[float, list[SweepResult]]:
    """Return the coverage cost and ordered cell sweeps in the sweep frame."""

    swept = rotate_polygon(polygon, -angle_rad)
    cells = decompose_polygon(swept, HORIZONTAL)
    start_swept = rotate_point(start_point, -angle_rad) if start_point is not None else None
    sweeps = [sweep_cell(cell, params) for cell in order_cells(cells, start_swept)]
    straight_length = 0.0
    for sweep in sweeps:
        straight_length += sweep.straight_length_m
    return straight_length + turn_penalty_m * _count_turns(sweeps), sweeps


def _count_turns(sweeps: list[SweepResult]) -> int:
    return sum(sweep.turn_count for sweep in sweeps) + max(0, len(sweeps) - 1)
