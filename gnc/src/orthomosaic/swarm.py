"""Divide an orthomosaic sweep among drones surveying the same area."""

from __future__ import annotations

import math
from dataclasses import replace

from shapely.errors import ShapelyError

from utils.src.coordinates import coordinate_to_enu, rotate_point
from utils.src.types import Coordinate

from .models import DroneSpec, ScanRequest
from .pathing import _plan_sweep, plan_orthomosaic_path
from .sweep import CapturePass


def plan_swarm_orthomosaic(
    request: ScanRequest, drones: list[DroneSpec]
) -> tuple[bool, dict[str, list[Coordinate]] | None]:
    """Assign contiguous groups of survey passes to drones.

    `request` supplies the survey boundary and camera settings. The `drones`
    list supplies the fleet size, unique non-blank IDs, and optional starts.

    Return: `(True, setpoints_by_id)` with ordered GPS photo setpoints at the
    derived survey altitude in metres, or `(False, None)` on invalid input or
    failed sweep generation. A drone without an assigned pass gets an empty list.
    """

    drone_ids = {drone.drone_id for drone in drones}
    if (
        not drones
        or len(drone_ids) != len(drones)
        or any(not name.strip() for name in drone_ids)
    ):
        return False, None
    for drone in drones:
        if drone.start is not None and not (
            -90.0 <= drone.start.lat <= 90.0 and -180.0 <= drone.start.lon <= 180.0
        ):
            return False, None

    try:
        if len(drones) == 1:
            drone = drones[0]
            plan = plan_orthomosaic_path(replace(request, start=drone.start))
            return True, {drone.drone_id: plan.capture_waypoints}

        survey = _plan_sweep(replace(request, start=None))
        passes = [
            sweep_pass for cell in survey.cells for sweep_pass in cell.capture_passes
        ]
        segments = _partition_passes(passes, len(drones))
        starts = {
            drone.drone_id: rotate_point(
                coordinate_to_enu(survey.anchor, drone.start),
                -math.radians(survey.angle_deg),
            )
            for drone in drones
            if drone.start is not None
        }
        assignments = _assign_segments(
            [drone.drone_id for drone in drones], starts, segments
        )
        return True, {
            drone_id: survey.to_coordinates(points)
            for drone_id, points in assignments.items()
        }
    except (ShapelyError, ValueError, OverflowError):
        return False, None


def _partition_passes(
    passes: list[CapturePass], drone_count: int
) -> list[list[tuple[float, float]]]:
    """Balance straight-line distance across nonempty groups of whole passes."""

    if len(passes) <= drone_count:
        return [sweep_pass.capture_waypoints for sweep_pass in passes]

    lengths = [sweep_pass.straight_length_m for sweep_pass in passes]
    lower = max(lengths)
    upper = sum(lengths)
    # A capacity is feasible if filling each group before opening the next fits.
    while upper - lower > 1e-9 * max(1.0, upper):
        capacity = (lower + upper) / 2.0
        if _groups_needed(lengths, capacity) <= drone_count:
            upper = capacity
        else:
            lower = capacity

    cuts = [0]
    distance = 0.0
    for index, length in enumerate(lengths):
        reserve_last_passes = len(passes) - index == drone_count - len(cuts)
        if index > cuts[-1] and (distance + length > upper or reserve_last_passes):
            cuts.append(index)
            distance = 0.0
        distance += length
    cuts.append(len(passes))
    return [
        [
            point
            for sweep_pass in passes[first:last]
            for point in sweep_pass.capture_waypoints
        ]
        for first, last in zip(cuts, cuts[1:])
    ]


def _groups_needed(lengths: list[float], capacity: float) -> int:
    groups = 1
    distance = 0.0
    for length in lengths:
        if distance + length > capacity:
            groups += 1
            distance = 0.0
        distance += length
    return groups


def _assign_segments(
    drone_ids: list[str],
    starts: dict[str, tuple[float, float]],
    segments: list[list[tuple[float, float]]],
) -> dict[str, list[tuple[float, float]]]:
    assignments: dict[str, list[tuple[float, float]]] = {
        drone_id: [] for drone_id in drone_ids
    }
    candidates: list[tuple[float, str, int, bool]] = []
    for drone_id, start in starts.items():
        for index, segment in enumerate(segments):
            entry_distance = math.dist(start, segment[0])
            exit_distance = math.dist(start, segment[-1])
            candidates.append(
                (
                    min(entry_distance, exit_distance),
                    drone_id,
                    index,
                    exit_distance < entry_distance,
                )
            )

    used: set[int] = set()
    for _, drone_id, index, reverse in sorted(candidates):
        if not assignments[drone_id] and index not in used:
            assignments[drone_id] = (
                segments[index][::-1] if reverse else segments[index]
            )
            used.add(index)

    remaining = (segment for index, segment in enumerate(segments) if index not in used)
    idle_drones = sorted(
        drone_id for drone_id in assignments if not assignments[drone_id]
    )
    for drone_id, segment in zip(idle_drones, remaining):
        assignments[drone_id] = segment
    return assignments
