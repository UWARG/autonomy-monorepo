"""Geometry types and conversions shared across the mock drone modules."""

from __future__ import annotations

import math
from typing import NamedTuple

# (east, north, up) metres, or (roll, pitch, yaw) radians for orientation.
Vector3 = tuple[float, float, float]

EARTH_RADIUS_M = 6_371_000.0  # mean radius


class GeoPosition(NamedTuple):
    """Degrees latitude/longitude, and altitude in metres above takeoff."""

    latitude: float
    longitude: float
    altitude: float


def to_local(origin: GeoPosition, point: GeoPosition) -> Vector3:
    """`point` as (east, north, up) metres from `origin`.

    Uses a flat-earth (equirectangular) approximation, accurate to centimetres over
    a few km. Not valid near the poles or across the ±180° longitude line.
    """
    east = (
        math.radians(point.longitude - origin.longitude)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin.latitude))
    )
    north = math.radians(point.latitude - origin.latitude) * EARTH_RADIUS_M
    return (east, north, point.altitude - origin.altitude)


def from_local(origin: GeoPosition, offset: Vector3) -> GeoPosition:
    """The position `offset` (east, north, up) metres from `origin`; inverse of to_local."""
    east, north, up = offset
    return GeoPosition(
        latitude=origin.latitude + math.degrees(north / EARTH_RADIUS_M),
        longitude=origin.longitude
        + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(origin.latitude)))
        ),
        altitude=origin.altitude + up,
    )
