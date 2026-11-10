"""Per-cell coverage generation and orientation cost for scan planning.

A cell is swept with parallel passes spaced by ``line_spacing_m``. Each pass
covers the exact x-extent of the cell within that pass's cross-track swath
band (``[y - swath/2, y + swath/2]``), which guarantees the pass covers every
cell point whose cross-track coordinate lies in its band. Passes are placed so
their bands tile the cell's cross-track extent with overlap, so every cell
point is covered by at least one photo.

Coverage is complete (this is a geometric guarantee for the supported cell
class). What is NOT guaranteed is row-to-row photo overlap at sloped cell
edges, because adjacent passes may have different lengths; that is a stitching
quality concern, not a coverage one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from shapely import affinity
from shapely.geometry import Polygon, box

from .decompose import HORIZONTAL, Cell

_DEDUPE_TOLERANCE_M = 1e-6


@dataclass(frozen=True)
class SweepParams:
    """Camera-derived coverage geometry for one sweep."""

    swath_m: float
    frame_m: float
    line_spacing_m: float
    capture_spacing_m: float


@dataclass(frozen=True)
class SweepResult:
    """Coverage waypoints and cost statistics for one cell."""

    capture_waypoints: list[tuple[float, float]]
    path_waypoints: list[tuple[float, float]]
    straight_length_m: float
    turn_count: int
    path_length_m: float


def default_turn_penalty_m(line_spacing_m: float) -> float:
    """Default per-turn cost in equivalent straight-line meters.

    A 180-degree turn between adjacent passes travels roughly a semicircle of
    radius ``line_spacing_m / 2``, i.e. ``pi * line_spacing_m / 2``. Real turns
    cost more than their arc length (deceleration, re-acceleration, no useful
    sensing), so the default charges twice the arc.
    """

    return math.pi * line_spacing_m


def coverage_cost(result: SweepResult, turn_penalty_m: float) -> float:
    """The orientation cost: productive straight length plus turn cost."""

    return result.straight_length_m + turn_penalty_m * result.turn_count


def sweep_cell(cell: Cell, params: SweepParams) -> SweepResult:
    """Generates coverage waypoints for a monotone ``cell``."""

    _validate_params(params)
    if cell.axis == HORIZONTAL:
        capture, path, straight, turns, path_length = _sweep_aligned(cell.polygon, params)
        return SweepResult(
            capture_waypoints=capture,
            path_waypoints=path,
            straight_length_m=straight,
            turn_count=turns,
            path_length_m=path_length,
        )

    swapped = affinity.affine_transform(cell.polygon, [0, 1, 1, 0, 0, 0])
    capture_swapped, path_swapped, straight, turns, path_length = _sweep_aligned(
        swapped, params
    )
    capture_waypoints = [(y, x) for (x, y) in capture_swapped]
    path_waypoints = [(y, x) for (x, y) in path_swapped]
    return SweepResult(
        capture_waypoints=capture_waypoints,
        path_waypoints=path_waypoints,
        straight_length_m=straight,
        turn_count=turns,
        path_length_m=path_length,
    )


def _validate_params(params: SweepParams) -> None:
    if params.swath_m <= 0.0 or params.frame_m <= 0.0:
        raise ValueError("swath_m and frame_m must be positive")
    if not 0.0 < params.line_spacing_m <= params.swath_m:
        raise ValueError("line_spacing_m must be in (0, swath_m]")
    if not 0.0 < params.capture_spacing_m <= params.frame_m:
        raise ValueError("capture_spacing_m must be in (0, frame_m]")


def _sweep_aligned(
    polygon: Polygon, params: SweepParams
) -> tuple[list[tuple[float, float]], list[tuple[float, float]], float, int, float]:
    """Sweeps a polygon with passes along x, stacked in y."""

    xmin, ymin, xmax, ymax = polygon.bounds
    swath = params.swath_m
    frame = params.frame_m
    line_spacing = params.line_spacing_m
    capture_spacing = params.capture_spacing_m

    height = ymax - ymin
    if height <= 0.0:
        return [], [], 0.0, 0, 0.0

    n_passes = 1 + math.ceil(max(0.0, height - swath) / line_spacing)
    pass_heights = [ymin + swath / 2.0 + index * line_spacing for index in range(n_passes)]

    capture_waypoints: list[tuple[float, float]] = []
    path_waypoints: list[tuple[float, float]] = []
    straight_length = 0.0

    for index, height_center in enumerate(pass_heights):
        band_bottom = max(ymin, height_center - swath / 2.0)
        band_top = min(ymax, height_center + swath / 2.0)
        region_in_band = polygon.intersection(box(xmin, band_bottom, xmax, band_top))
        if region_in_band.is_empty:
            continue
        left, _, right, _ = region_in_band.bounds

        xs = _capture_positions(left, right, frame, capture_spacing)
        straight_length += right - left

        start = (left, height_center)
        end = (right, height_center)
        pass_captures = [(x, height_center) for x in xs]

        if index % 2 == 0:
            points = [start] + pass_captures + [end]
        else:
            points = [end] + list(reversed(pass_captures)) + [start]
            pass_captures = list(reversed(pass_captures))

        capture_waypoints.extend(pass_captures)
        path_waypoints.extend(points)

    path_waypoints = _dedupe(path_waypoints)
    capture_waypoints = _dedupe(capture_waypoints)
    turn_count = max(0, n_passes - 1)
    return capture_waypoints, path_waypoints, straight_length, turn_count, path_length(
        path_waypoints
    )


def _capture_positions(
    left: float, right: float, frame: float, capture_spacing: float
) -> list[float]:
    """Returns x-positions along ``[left, right]`` whose footprints tile it."""

    length = right - left
    if length <= frame:
        return [(left + right) / 2.0]

    count = math.ceil((length - frame) / capture_spacing) + 1
    xs = [left + frame / 2.0 + index * capture_spacing for index in range(count)]
    limit = right - frame / 2.0
    if xs[-1] > limit:
        xs[-1] = limit
    return xs


def _dedupe(
    points: list[tuple[float, float]], tolerance: float = _DEDUPE_TOLERANCE_M
) -> list[tuple[float, float]]:
    """Drops consecutive points closer than ``tolerance`` meters."""

    result: list[tuple[float, float]] = []
    for point in points:
        if not result:
            result.append(point)
            continue
        if math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > tolerance:
            result.append(point)
    return result


def path_length(points: list[tuple[float, float]]) -> float:
    """Total distance along a polyline through ``points``."""

    return sum(
        math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(points, points[1:])
    )