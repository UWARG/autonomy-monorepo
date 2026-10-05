"""Boustrophedon cell decomposition for orthomosaic coverage planning.

A region can be covered by a single back-and-forth (lawn-mower) sweep iff
every slice perpendicular to the pass direction intersects it in exactly one
interval; such a region is ``monotone``. Concave regions are decomposed into
monotone cells by cutting them at ``critical heights``: heights where the
slice interval count changes. Those heights are the y-coordinates (for a
horizontal sweep) of reflex vertices that are local extrema in the sweep
direction.

Within a band between consecutive critical heights the slice count is
constant, and a *connected* piece of a band can only have count one: two
same-height intervals inside a connected piece would have to merge at some
height inside the band, which would itself be a critical height. Hence each
cell of a simple polygon without holes is monotone, and the cells tile the
polygon exactly. This correctness argument holds for the supported class:
simple polygons without holes.

Cells are formed by intersecting the polygon with horizontal/vertical band
rectangles (not ``shapely.ops.split``, whose behaviour when a cut line
touches a vertex is fragile). If a cell is somehow not monotone (input
outside the supported class), it is re-decomposed along the other axis. That
fallback is NOT guaranteed to converge for arbitrary polygons, so recursion
depth is capped and an error is raised if it does not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from shapely.geometry import LineString, Polygon, box
from shapely.geometry.polygon import orient

HORIZONTAL = 0
VERTICAL = 1

_MAX_DEPTH = 8


@dataclass(frozen=True)
class Cell:
    """A monotone region and the axis along which it should be swept."""

    polygon: Polygon
    axis: int


def validate_polygon(polygon: Polygon) -> None:
    """Raises ``ValueError`` if ``polygon`` is outside the supported class.

    Supported: a single, valid, non-degenerate simple polygon with no holes.
    """

    if not isinstance(polygon, Polygon):
        raise ValueError("boundary must be a single polygon")
    if polygon.is_empty:
        raise ValueError("boundary polygon is empty")
    if not polygon.is_valid:
        raise ValueError("boundary polygon is self-intersecting or invalid")
    if polygon.interiors:
        raise ValueError("boundary polygon must not contain holes")
    if polygon.area <= 0.0:
        raise ValueError("boundary polygon has zero area")


def is_monotone(polygon: Polygon, axis: int, samples: int = 8) -> bool:
    """True if every slice of ``polygon`` along ``axis`` is a single interval.

    For ``HORIZONTAL``, slices are horizontal lines; for ``VERTICAL``, slices
    are vertical lines.
    """

    low, high = _slice_bounds(polygon, axis)
    for index in range(1, samples):
        position = low + (high - low) * index / samples
        if _slice_count(polygon, axis, position) > 1:
            return False
    return True


def decompose_polygon(polygon: Polygon, axis: int = HORIZONTAL) -> list[Cell]:
    """Decomposes ``polygon`` into monotone boustrophedon cells.

    Cells are ordered along the sweep axis but are not otherwise sequenced;
    ordering between cells is the caller's responsibility.
    """

    polygon = orient(polygon, sign=1.0)
    validate_polygon(polygon)
    return _decompose(polygon, axis, depth=0)


def order_cells(
    cells: list[Cell], reference_point: tuple[float, float] | None = None
) -> list[Cell]:
    """Orders cells by greedy nearest-neighbour, starting near ``reference_point``.

    The first cell is the one whose centroid is nearest to ``reference_point``
    (or the origin when it is None); each following cell is the unvisited cell
    nearest to the current one.
    """

    if not cells:
        return []

    centroids = [
        (cell.polygon.centroid.x, cell.polygon.centroid.y) for cell in cells
    ]
    reference = reference_point if reference_point is not None else (0.0, 0.0)

    def squared_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

    first = min(
        range(len(cells)),
        key=lambda index: (squared_distance(reference, centroids[index]), index),
    )
    ordered = [cells[first]]
    remaining = set(range(len(cells)))
    remaining.discard(first)
    current = first

    while remaining:
        nxt = min(
            remaining,
            key=lambda index: (
                squared_distance(centroids[current], centroids[index]),
                index,
            ),
        )
        ordered.append(cells[nxt])
        remaining.discard(nxt)
        current = nxt

    return ordered


def _decompose(polygon: Polygon, axis: int, depth: int) -> list[Cell]:
    if depth > _MAX_DEPTH:
        raise ValueError(
            "polygon decomposition did not converge; the boundary may be "
            "outside the supported class (simple polygon, no holes)"
        )

    polygon = orient(polygon, sign=1.0)

    positions = _critical_positions(polygon, axis)
    if positions:
        cells: list[Cell] = []
        for part in _split_into_bands(polygon, axis, positions):
            cells.extend(_decompose(part, axis, depth + 1))
        return cells

    if is_monotone(polygon, axis):
        return [Cell(polygon, axis)]

    return _decompose(polygon, 1 - axis, depth + 1)


def _slice_bounds(polygon: Polygon, axis: int) -> tuple[float, float]:
    xmin, ymin, xmax, ymax = polygon.bounds
    if axis == HORIZONTAL:
        return ymin, ymax
    return xmin, xmax


def _slice_count(polygon: Polygon, axis: int, position: float) -> int:
    """Returns the number of intervals in the slice of ``polygon`` at position."""

    xmin, ymin, xmax, ymax = polygon.bounds
    if axis == HORIZONTAL:
        line = LineString([(xmin, position), (xmax, position)])
    else:
        line = LineString([(position, ymin), (position, ymax)])
    return _count_intervals(polygon.intersection(line))


def _count_intervals(geometry) -> int:
    """Counts LineString parts, ignoring points that arise from tangency."""

    if geometry.is_empty:
        return 0
    geometry_type = geometry.geom_type
    if geometry_type == "LineString":
        return 1
    if geometry_type == "MultiLineString":
        return len(geometry.geoms)
    if geometry_type in ("GeometryCollection", "MultiPoint"):
        return sum(_count_intervals(part) for part in geometry.geoms)
    return 0


def _reflex_vertices(polygon: Polygon) -> list[tuple[float, float]]:
    """Returns the reflex (concave) vertices of a CCW polygon."""

    coordinates = list(polygon.exterior.coords)[:-1]
    reflex = []
    for index in range(len(coordinates)):
        previous = coordinates[index - 1]
        current = coordinates[index]
        following = coordinates[(index + 1) % len(coordinates)]
        ax, ay = current[0] - previous[0], current[1] - previous[1]
        bx, by = following[0] - current[0], following[1] - current[1]
        cross = ax * by - ay * bx
        tolerance = 1e-9 * math.hypot(ax, ay) * math.hypot(bx, by)
        if cross < -tolerance:
            reflex.append(current)
    return reflex


def _critical_positions(polygon: Polygon, axis: int) -> list[float]:
    """Returns sorted critical heights where the slice count changes."""

    axis_index = 1 if axis == HORIZONTAL else 0
    epsilon = _slice_epsilon(polygon, axis)
    positions = []
    for vertex in _reflex_vertices(polygon):
        position = vertex[axis_index]
        below = _slice_count(polygon, axis, position - epsilon)
        above = _slice_count(polygon, axis, position + epsilon)
        if below != above:
            positions.append(position)
    return sorted(set(positions))


def _slice_epsilon(polygon: Polygon, axis: int) -> float:
    """A step smaller than the gap between any two vertex positions on the axis."""

    axis_index = 1 if axis == HORIZONTAL else 0
    values = sorted({coordinate[axis_index] for coordinate in polygon.exterior.coords})
    if len(values) < 2:
        return 1e-9
    minimum_gap = min(
        values[index + 1] - values[index] for index in range(len(values) - 1)
    )
    if minimum_gap <= 0.0:
        return 1e-9
    return minimum_gap * 0.25


def _split_into_bands(
    polygon: Polygon, axis: int, positions: list[float]
) -> list[Polygon]:
    """Splits ``polygon`` into monotone pieces with band rectangles."""

    xmin, ymin, xmax, ymax = polygon.bounds
    if axis == HORIZONTAL:
        bounds = [ymin] + positions + [ymax]
    else:
        bounds = [xmin] + positions + [xmax]

    minimum_area = polygon.area * 1e-9
    pieces: list[Polygon] = []
    for low, high in zip(bounds, bounds[1:]):
        if high - low <= 1e-12:
            continue
        if axis == HORIZONTAL:
            band = box(xmin, low, xmax, high)
        else:
            band = box(low, ymin, high, ymax)
        for part in _as_polygons(polygon.intersection(band)):
            if part.area > minimum_area:
                pieces.append(part)
    return pieces


def _as_polygons(geometry) -> list[Polygon]:
    if geometry.is_empty:
        return []
    geometry_type = geometry.geom_type
    if geometry_type == "Polygon":
        return [geometry]
    if geometry_type == "MultiPolygon":
        return list(geometry.geoms)
    return [part for part in geometry.geoms if part.geom_type == "Polygon"]