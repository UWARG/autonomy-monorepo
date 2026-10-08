"""
Decoding of the AEAC competition server's ``traffic`` event.
"""

from __future__ import annotations

import dataclasses
import json
import math
from typing import Any


TRAFFIC_EVENT = "traffic"
ERROR_EVENT = "error"


@dataclasses.dataclass(frozen=True)
class TrafficAircraft:
    """
    One simulated aircraft from a traffic snapshot.
    """

    aircraft_index: int
    name: str
    horizontal_keep_away_m: float
    vertical_keep_away_m: float
    lat: float
    lon: float
    altitude_agl_m: float
    speed_mps: float
    direction_deg: float


def decode_message(raw: str | bytes) -> dict[str, Any] | None:
    """
    Parses one message from the server, or returns None if it is not a JSON
    object.
    """

    try:
        message = json.loads(raw)
    except (TypeError, ValueError):
        return None
    return message if isinstance(message, dict) else None


def _finite_number(mapping: dict[str, Any], key: str) -> float:
    value = mapping[key]
    # bool is an int in Python, but is not a number here
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"'{key}' is not a number: {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"'{key}' is not finite: {value!r}")
    return float(value)


def _aircraft_from_entry(entry: Any) -> TrafficAircraft:
    """
    Raises ``KeyError``, ``TypeError`` or ``ValueError`` if ``entry`` is not a
    complete, sensible aircraft.
    """

    position = entry["position"]

    aircraft_index = entry["aircraftIndex"]
    if isinstance(aircraft_index, bool) or not isinstance(aircraft_index, int):
        raise ValueError(f"'aircraftIndex' is not an integer: {aircraft_index!r}")
    # Has to fit the uint8 it is published as
    if not 0 <= aircraft_index <= 255:
        raise ValueError(f"'aircraftIndex' out of range: {aircraft_index}")

    lat = _finite_number(position, "lat")
    lon = _finite_number(position, "lon")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError(f"position out of range: lat={lat} lon={lon}")

    return TrafficAircraft(
        aircraft_index=aircraft_index,
        name=str(entry.get("name", "")),
        horizontal_keep_away_m=_finite_number(entry, "horizontalKeepAway"),
        vertical_keep_away_m=_finite_number(entry, "verticalKeepAway"),
        lat=lat,
        lon=lon,
        altitude_agl_m=_finite_number(position, "altitude"),
        speed_mps=_finite_number(position, "speed"),
        direction_deg=_finite_number(position, "direction"),
    )


def parse_traffic(message: dict[str, Any]) -> tuple[list[TrafficAircraft], list[str]]:
    """
    Reads the aircraft out of a ``traffic`` event.

    Returns ``(aircraft, problems)``. An aircraft that is missing fields or has
    nonsense values is left out and described in ``problems`` instead, so one
    bad entry does not cost the rest of the snapshot.
    """

    payload = message.get("payload")
    entries = payload.get("traffic") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        return [], ["traffic event has no 'payload.traffic' list"]

    aircraft = []
    problems = []
    for position, entry in enumerate(entries):
        try:
            aircraft.append(_aircraft_from_entry(entry))
        except (KeyError, TypeError, ValueError) as error:
            problems.append(f"traffic entry {position} skipped: {error!r}")
    return aircraft, problems
