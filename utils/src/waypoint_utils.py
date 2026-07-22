"""
Parsing and ordering utils of lap waypoints.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from .constants import EARTH_RADIUS_M, TWO_PI
from .types import Coordinate


def east_north_coordinate_offset_m(
    from_lat: float, from_lon: float, to_lat: float, to_lon: float
) -> tuple[float, float]:
    """
    Calculates the ``(east, north)`` offset in meters from one point
    to another.
    """

    east = (
        math.radians(to_lon - from_lon)
        * math.cos(math.radians((from_lat + to_lat) / 2.0))
        * EARTH_RADIUS_M
    )
    north = math.radians(to_lat - from_lat) * EARTH_RADIUS_M
    return east, north


def _coordinate_from_entry(entry: Any, path: str | Path, context: str) -> Coordinate:
    """
    Builds a :class:`Coordinate` from a parsed YAML mapping.

    Raises ``ValueError`` if ``entry`` is not a ``lat/lon/alt``
    mapping, has a non-numeric value, or has out of range coordinates.
    """

    if not isinstance(entry, dict):
        raise ValueError(
            f"{path}: {context} must be a mapping with 'lat', 'lon', and "
            f"'alt' keys, got {entry!r}"
        )

    missing = [key for key in ("lat", "lon", "alt") if key not in entry]
    if missing:
        raise ValueError(
            f"{path}: {context} is missing key(s) {', '.join(missing)} in "
            f"{entry!r}"
        )

    try:
        coordinate = Coordinate(
            float(entry["lat"]), float(entry["lon"]), float(entry["alt"])
        )
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{path}: {context} has a non-numeric value in {entry!r}"
        ) from error

    if not (-90.0 <= coordinate.lat <= 90.0 and -180.0 <= coordinate.lon <= 180.0):
        raise ValueError(
            f"{path}: {context} latitude/longitude out of range in {entry!r}"
        )

    return coordinate


def parse_waypoints_file(
    path: str | Path,
) -> tuple[Coordinate | None, list[Coordinate]]:
    """
    Parses a YAML waypoints file into ``(home, lap_waypoints)``.

    The file is a mapping with an optional ``home`` coordinate and a
    ``waypoints`` list, each waypoint a mapping of ``lat``, ``lon``, and
    ``alt``::

        home:
          lat: 43.471520
          lon: -80.541400
          alt: 15
        waypoints:
          - {lat: 43.471790, lon: -80.541524, alt: 15}

    ``home`` is None if omitted. Raises ``OSError`` if the file cannot be
    read and ``ValueError`` if its contents are malformed.
    """

    text = Path(path).read_text()
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ValueError(f"{path}: invalid YAML: {error}") from error

    if raw is None:
        return None, []
    if not isinstance(raw, dict):
        raise ValueError(
            f"{path}: expected a mapping with 'home' and 'waypoints', got {raw!r}"
        )

    home_entry = raw.get("home")
    home = (
        _coordinate_from_entry(home_entry, path, "home")
        if home_entry is not None
        else None
    )

    waypoints_entry = raw.get("waypoints") or []
    if not isinstance(waypoints_entry, list):
        raise ValueError(
            f"{path}: 'waypoints' must be a list, got {waypoints_entry!r}"
        )

    waypoints = [
        _coordinate_from_entry(entry, path, f"waypoint {index}")
        for index, entry in enumerate(waypoints_entry, start=1)
    ]

    return home, waypoints


def sort_clockwise_sweep(
    waypoints: list[Coordinate], home: Coordinate | None = None
) -> list[Coordinate]:
    """
    Orders waypoints as a clockwise circular sweep around their centroid
    starting from the home's direction or North if home is None.
    """

    if len(waypoints) <= 1:
        return list(waypoints)

    centroid_lat = sum(waypoint.lat for waypoint in waypoints) / len(waypoints)
    centroid_lon = sum(waypoint.lon for waypoint in waypoints) / len(waypoints)

    def bearing_from_centroid(waypoint: Coordinate) -> float:
        east, north = east_north_coordinate_offset_m(
            centroid_lat, centroid_lon, waypoint.lat, waypoint.lon
        )
        # atan2(east, north) is the compass bearing: 0 at north, increasing
        # clockwise, normalized to [0, 2*pi)
        return math.atan2(east, north) % TWO_PI

    start_bearing = 0.0
    if home is not None and not math.isclose(
        math.hypot(*east_north_coordinate_offset_m(centroid_lat, centroid_lon, home.lat, home.lon)),
        0.0,
        abs_tol=1e-9,
    ):
        start_bearing = bearing_from_centroid(home)

    def sweep_key(waypoint: Coordinate) -> tuple[float, float]:
        relative_bearing = (bearing_from_centroid(waypoint) - start_bearing) % TWO_PI
        distance = math.hypot(
            *east_north_coordinate_offset_m(centroid_lat, centroid_lon, waypoint.lat, waypoint.lon)
        )
        return (relative_bearing, distance)

    return sorted(waypoints, key=sweep_key)
