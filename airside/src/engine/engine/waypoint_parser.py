"""
Parsing and ordering of lap waypoints.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import NamedTuple

TWO_PI = 2.0 * math.pi
EARTH_RADIUS_M = 6371008.8


class Waypoint(NamedTuple):
    lat: float  # WGS84 latitude, decimal degrees
    lon: float  # WGS84 longitude, decimal degrees
    alt: float  # meters above the home/takeoff position


def enu_offset_m(
    from_lat: float, from_lon: float, to_lat: float, to_lon: float
) -> tuple[float, float]:
    """
    Calculates the ``(east, north)`` offset in meters from one WGS84 point
    to another.
    """

    east = (
        math.radians(to_lon - from_lon)
        * math.cos(math.radians((from_lat + to_lat) / 2.0))
        * EARTH_RADIUS_M
    )
    north = math.radians(to_lat - from_lat) * EARTH_RADIUS_M
    return east, north


def parse_waypoints_file(path: str | Path) -> tuple[Waypoint | None, list[Waypoint]]:
    """
    Parses a waypoints file into ``(home, lap_waypoints)``.

    ``home`` is the waypoint marked with a leading ``h``, or None if no line
    is marked. Raises ``OSError`` if the file cannot be read and
    ``ValueError`` if a line is malformed or more than one home is marked.
    """

    home: Waypoint | None = None
    waypoints: list[Waypoint] = []
    for line_number, raw_line in enumerate(
        Path(path).read_text().splitlines(), start=1
    ):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue

        parts = line.replace(",", " ").split()
        is_home = parts and parts[0].lower() == "h"
        if is_home:
            parts = parts[1:]

        if len(parts) != 3:
            raise ValueError(
                f"{path}:{line_number}: expected 'lat, lon, alt' or "
                f"'h lat, lon, alt', got {raw_line!r}"
            )
        try:
            waypoint = Waypoint(*(float(part) for part in parts))
        except ValueError as error:
            raise ValueError(
                f"{path}:{line_number}: non-numeric value in {raw_line!r}"
            ) from error

        if not (-90.0 <= waypoint.lat <= 90.0 and -180.0 <= waypoint.lon <= 180.0):
            raise ValueError(
                f"{path}:{line_number}: latitude/longitude out of range in "
                f"{raw_line!r}"
            )

        if is_home:
            if home is not None:
                raise ValueError(
                    f"{path}:{line_number}: more than one home ('h') waypoint"
                )
            home = waypoint
        else:
            waypoints.append(waypoint)

    return home, waypoints


def sort_clockwise_sweep(
    waypoints: list[Waypoint], home: Waypoint | None = None
) -> list[Waypoint]:
    """
    Orders waypoints as a clockwise circular sweep around their centroid
    starting from the home's direction.
    """

    if len(waypoints) <= 1:
        return list(waypoints)

    centroid_lat = sum(waypoint.lat for waypoint in waypoints) / len(waypoints)
    centroid_lon = sum(waypoint.lon for waypoint in waypoints) / len(waypoints)

    def bearing_from_centroid(waypoint: Waypoint) -> float:
        east, north = enu_offset_m(
            centroid_lat, centroid_lon, waypoint.lat, waypoint.lon
        )
        # atan2(east, north) is the compass bearing: 0 at north, increasing
        # clockwise, normalized to [0, 2*pi)
        return math.atan2(east, north) % TWO_PI

    start_bearing = 0.0
    if home is not None and not math.isclose(
        math.hypot(*enu_offset_m(centroid_lat, centroid_lon, home.lat, home.lon)),
        0.0,
        abs_tol=1e-9,
    ):
        start_bearing = bearing_from_centroid(home)

    def sweep_key(waypoint: Waypoint) -> tuple[float, float]:
        relative_bearing = (bearing_from_centroid(waypoint) - start_bearing) % TWO_PI
        distance = math.hypot(
            *enu_offset_m(centroid_lat, centroid_lon, waypoint.lat, waypoint.lon)
        )
        return (relative_bearing, distance)

    return sorted(waypoints, key=sweep_key)
