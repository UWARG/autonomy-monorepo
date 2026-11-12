"""Local east/north (ENU) conversion and planar rotation helpers."""

from __future__ import annotations

import math
from collections.abc import Mapping

from shapely import affinity
from shapely.geometry import Polygon

from .constants import EARTH_RADIUS_M
from .types import Coordinate
from .waypoint_utils import east_north_coordinate_offset_m


def rotate_point(
    point: tuple[float, float], angle_rad: float
) -> tuple[float, float]:
    """Rotates ``(east, north)`` counterclockwise about the origin by angle."""

    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    x, y = point
    return (x * cosine - y * sine, x * sine + y * cosine)


def rotate_points(
    points: list[tuple[float, float]], angle_rad: float
) -> list[tuple[float, float]]:
    """Rotates every point counterclockwise about the origin by angle."""

    return [rotate_point(point, angle_rad) for point in points]


def rotate_polygon(polygon: Polygon, angle_rad: float) -> Polygon:
    """Rotates a shapely polygon counterclockwise about the origin by angle.

    Uses the same matrix as :func:`rotate_point`, so points and polygons
    always agree on the rotation direction.
    """

    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return affinity.affine_transform(polygon, [cosine, -sine, sine, cosine, 0.0, 0.0])


def anchor_for_boundary(boundary: list[Coordinate]) -> Coordinate:
    """Returns the boundary centroid, used as the local frame origin."""

    if not boundary:
        raise ValueError("boundary must not be empty")
    latitude = sum(point.lat for point in boundary) / len(boundary)
    longitude = sum(point.lon for point in boundary) / len(boundary)
    return Coordinate(latitude, longitude, 0.0)


def coordinate_to_enu(anchor: Coordinate, point: Coordinate) -> tuple[float, float]:
    """Converts a GPS coordinate to local ``(east, north)`` meters from anchor."""

    return east_north_coordinate_offset_m(anchor.lat, anchor.lon, point.lat, point.lon)


def boundary_to_enu(
    anchor: Coordinate, boundary: list[Coordinate]
) -> list[tuple[float, float]]:
    """Converts every boundary vertex to local ``(east, north)`` meters."""

    return [coordinate_to_enu(anchor, point) for point in boundary]


def enu_to_coordinate(
    anchor: Coordinate,
    east_m: float,
    north_m: float,
    alt_m: float = 0.0,
) -> Coordinate:
    """Converts local ``(east, north)`` meters back to a GPS coordinate.

    Latitude is recovered directly from the north offset (the forward north
    term is linear in latitude). Longitude is then recovered using the mean
    latitude of the anchor and target, mirroring the forward function's
    ``cos(mean_lat)`` term. This is an exact algebraic inverse.
    """

    if not math.isfinite(east_m) or not math.isfinite(north_m):
        raise ValueError("east_m and north_m must be finite")

    latitude = anchor.lat + math.degrees(north_m / EARTH_RADIUS_M)
    mean_latitude = math.radians((anchor.lat + latitude) / 2.0)
    longitude = anchor.lon + math.degrees(
        east_m / (math.cos(mean_latitude) * EARTH_RADIUS_M)
    )
    return Coordinate(latitude, longitude, alt_m)


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


def local_region(boundary: list[Coordinate]) -> tuple[Coordinate, Polygon]:
    """Returns the local frame anchor and the boundary as a validated ENU polygon."""

    if len(boundary) < 3:
        raise ValueError("boundary needs at least 3 vertices")
    anchor = anchor_for_boundary(boundary)
    region = Polygon(boundary_to_enu(anchor, boundary))
    validate_polygon(region)
    return anchor, region


def positions_to_enu(
    anchor: Coordinate, positions: Mapping[int, Coordinate]
) -> dict[int, tuple[float, float]]:
    """Converts GPS positions keyed by id to local ``(east, north)`` meters."""

    for position in positions.values():
        if not (math.isfinite(position.lat) and math.isfinite(position.lon)):
            raise ValueError("drone positions must be finite")
    return {
        drone_id: coordinate_to_enu(anchor, position)
        for drone_id, position in positions.items()
    }


def enu_to_positions(
    anchor: Coordinate,
    points: Mapping[int, tuple[float, float]],
    originals: Mapping[int, Coordinate],
) -> dict[int, Coordinate]:
    """Converts local points keyed by id back to GPS.

    Each point keeps the altitude of the position with the same id in
    ``originals``.
    """

    return {
        drone_id: enu_to_coordinate(anchor, east, north, originals[drone_id].alt)
        for drone_id, (east, north) in points.items()
    }