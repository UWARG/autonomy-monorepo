"""Lloyd's algorithm: spread drones evenly over a region by Voronoi relaxation.

Each step gives every drone the part of the region closer to it than to any
other drone (its Voronoi cell, clipped to the region) and moves the drone to
the centroid of that cell. Repeating this converges to a centroidal Voronoi
tessellation, where every drone covers a similar share of the area.

All geometry runs in local east/north metres around the boundary centroid,
so distances and areas are real metres even though the API takes GPS.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from shapely.geometry import MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, unary_union, voronoi_diagram

from utils.src.coordinates import enu_to_positions, local_region, positions_to_enu
from utils.src.types import Coordinate

LocalPoint = tuple[float, float]

# Two drones closer than this have no well-defined boundary between them.
MIN_SEPARATION_M = 1e-3


@dataclass(frozen=True)
class RelaxationResult:
    """Outcome of running Lloyd's algorithm until the drones stop moving."""

    positions: dict[int, Coordinate]
    iterations: int
    converged: bool
    history: list[dict[int, Coordinate]]


def lloyd_step(
    positions: Mapping[int, Coordinate], boundary: list[Coordinate]
) -> dict[int, Coordinate]:
    """Returns where each drone should go next, keyed by the same drone ids.

    ``positions`` maps drone id to its current GPS position; ``boundary`` is
    the region to spread over, as ordered GPS vertices. Each drone keeps its
    own altitude.
    """

    anchor, region = local_region(boundary)
    local = positions_to_enu(anchor, positions)
    moved = step_local(local, region)
    return enu_to_positions(anchor, moved, positions)


def lloyd_relax(
    positions: Mapping[int, Coordinate],
    boundary: list[Coordinate],
    max_iterations: int = 100,
    tolerance_m: float = 0.1,
) -> RelaxationResult:
    """Repeats :func:`lloyd_step` until no drone moves more than ``tolerance_m``.

    ``history`` starts with the input positions and holds one entry per step,
    so it can be replayed or plotted.
    """

    if max_iterations < 1:
        raise ValueError("max_iterations must be at least 1")
    if not math.isfinite(tolerance_m) or tolerance_m <= 0.0:
        raise ValueError("tolerance_m must be a positive number")

    anchor, region = local_region(boundary)
    local = positions_to_enu(anchor, positions)
    history = [dict(positions)]
    converged = False
    iterations = 0
    while iterations < max_iterations:
        moved = step_local(local, region)
        iterations += 1
        largest_move = max(math.dist(local[i], moved[i]) for i in local)
        local = moved
        history.append(enu_to_positions(anchor, local, positions))
        if largest_move < tolerance_m:
            converged = True
            break
    return RelaxationResult(history[-1], iterations, converged, history)


def coverage_cost(
    positions: Mapping[int, Coordinate], boundary: list[Coordinate]
) -> float:
    """Mean squared distance (m^2) from a point in the region to its nearest drone.

    Lower is better; Lloyd's algorithm never increases it.
    """

    anchor, region = local_region(boundary)
    return coverage_cost_local(positions_to_enu(anchor, positions), region)


def step_local(
    points: Mapping[int, LocalPoint], region: Polygon
) -> dict[int, LocalPoint]:
    """One Lloyd step in local metres: move each point to its cell's centroid."""

    cells = voronoi_cells(points, region)
    moved = {}
    stranded = []
    for drone_id, point in points.items():
        cell = cells[drone_id]
        if cell.is_empty:
            stranded.append(drone_id)
            continue
        target = cell.centroid
        if not cell.covers(target):
            # A cell cut by a concave boundary can have its centroid outside
            # it. The nearest point of the cell is the best position the
            # drone can hold while keeping its own cell.
            target = _nearest_point(cell, target)
        moved[drone_id] = (target.x, target.y)

    # Only a drone outside the region can lose its whole cell. Bring it back
    # to the nearest free point of the region, one at a time, so two stranded
    # drones near the same corner don't land on the same spot.
    for drone_id in stranded:
        taken = [Point(p).buffer(2 * MIN_SEPARATION_M) for p in moved.values()]
        free = region.difference(unary_union(taken)) if taken else region
        target = _nearest_point(free, Point(points[drone_id]))
        moved[drone_id] = (target.x, target.y)
    # Keep the caller's id order.
    return {drone_id: moved[drone_id] for drone_id in points}


def voronoi_cells(
    points: Mapping[int, LocalPoint], region: Polygon
) -> dict[int, BaseGeometry]:
    """Each point's Voronoi cell clipped to ``region`` (may be empty)."""

    _validate_points(points)
    if len(points) == 1:
        return {drone_id: region for drone_id in points}

    diagram = voronoi_diagram(MultiPoint(list(points.values())), envelope=region)
    unclipped = list(diagram.geoms)
    cells = {}
    for drone_id, point in points.items():
        # GEOS does not return cells in input order, so match each drone to
        # the cell that holds it (distance 0; min() also absorbs round-off).
        own = min(unclipped, key=Point(point).distance)
        cells[drone_id] = own.intersection(region)
    return cells


def coverage_cost_local(points: Mapping[int, LocalPoint], region: Polygon) -> float:
    """Mean squared distance (m^2) from a region point to its nearest drone."""

    cells = voronoi_cells(points, region)
    total = sum(
        _squared_distance_integral(cells[drone_id], points[drone_id])
        for drone_id in points
    )
    return total / region.area


def _squared_distance_integral(geometry: BaseGeometry, point: LocalPoint) -> float:
    """Integral of |q - point|^2 over the area of ``geometry``, exactly."""

    if geometry.is_empty:
        return 0.0
    if hasattr(geometry, "geoms"):
        return sum(_squared_distance_integral(part, point) for part in geometry.geoms)
    if not isinstance(geometry, Polygon):
        return 0.0  # points and lines left over from clipping have no area
    total = _ring_integral(list(geometry.exterior.coords), point)
    for hole in geometry.interiors:
        total -= _ring_integral(list(hole.coords), point)
    return total


def _ring_integral(coords: list[LocalPoint], point: LocalPoint) -> float:
    """Integral of |q - point|^2 over a simple ring (polygon second moment)."""

    px, py = point
    shifted = [(x - px, y - py) for x, y in coords]
    total = 0.0
    for (x0, y0), (x1, y1) in zip(shifted, shifted[1:]):
        cross = x0 * y1 - x1 * y0
        total += cross * (x0 * x0 + x0 * x1 + x1 * x1 + y0 * y0 + y0 * y1 + y1 * y1)
    # Ring direction only flips the sign.
    return abs(total) / 12.0


def _nearest_point(geometry: BaseGeometry, point: Point) -> Point:
    return nearest_points(geometry, point)[0]


def _validate_points(points: Mapping[int, LocalPoint]) -> None:
    if not points:
        raise ValueError("at least one drone position is required")
    for x, y in points.values():
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("drone positions must be finite")
    values = list(points.values())
    for i, first in enumerate(values):
        for second in values[i + 1 :]:
            if math.dist(first, second) < MIN_SEPARATION_M:
                raise ValueError("two drones share the same position")

