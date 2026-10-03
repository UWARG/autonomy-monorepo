"""Local east/north (ENU) conversion for scan-planning geometry."""

from __future__ import annotations

import math

from shapely import affinity
from shapely.geometry import Polygon

from utils.src.constants import EARTH_RADIUS_M
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m


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