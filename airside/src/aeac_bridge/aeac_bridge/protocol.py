"""Strict, dependency-free parsing for AEAC traffic events."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any


class TrafficProtocolError(ValueError):
    """Raised when a traffic event cannot be used safely."""


@dataclass(frozen=True, slots=True)
class TrafficAircraftData:
    aircraft_index: int
    name: str
    latitude_deg: float
    longitude_deg: float
    altitude_agl_m: float
    speed_mps: float
    heading_deg_true: float
    horizontal_keepaway_m: float
    vertical_keepaway_m: float


@dataclass(frozen=True, slots=True)
class TrafficEvent:
    aircraft: tuple[TrafficAircraftData, ...]


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TrafficProtocolError(f"{field} must be an object")
    return value


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TrafficProtocolError(f"{field} must be numeric")
    converted = float(value)
    if not math.isfinite(converted):
        raise TrafficProtocolError(f"{field} must be finite")
    return converted


def _positive_number(value: Any, field: str) -> float:
    converted = _finite_number(value, field)
    if converted <= 0.0:
        raise TrafficProtocolError(f"{field} must be positive")
    return converted


def parse_traffic_event(raw: str | bytes | bytearray) -> TrafficEvent | None:
    """Parse one AEAC envelope; return ``None`` for a valid non-traffic event."""

    try:
        message = json.loads(raw)
    except (TypeError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise TrafficProtocolError("message must be valid JSON") from error

    envelope = _mapping(message, "message")
    event = envelope.get("event")
    if not isinstance(event, str) or not event:
        raise TrafficProtocolError("event must be a non-empty string")
    if event != "traffic":
        return None

    payload = _mapping(envelope.get("payload"), "payload")
    raw_aircraft = payload.get("traffic")
    if not isinstance(raw_aircraft, list):
        raise TrafficProtocolError("payload.traffic must be a list")

    parsed: list[TrafficAircraftData] = []
    seen_indices: set[int] = set()
    for index, raw_entry in enumerate(raw_aircraft):
        prefix = f"payload.traffic[{index}]"
        entry = _mapping(raw_entry, prefix)
        aircraft_index = entry.get("aircraftIndex")
        if (
            isinstance(aircraft_index, bool)
            or not isinstance(aircraft_index, int)
            or aircraft_index < 0
        ):
            raise TrafficProtocolError(
                f"{prefix}.aircraftIndex must be a non-negative integer"
            )
        if aircraft_index in seen_indices:
            raise TrafficProtocolError("aircraftIndex values must be unique")
        seen_indices.add(aircraft_index)

        name = entry.get("name")
        if not isinstance(name, str) or not name.strip():
            raise TrafficProtocolError(f"{prefix}.name must be a non-empty string")

        position = _mapping(entry.get("position"), f"{prefix}.position")
        latitude_deg = _finite_number(position.get("lat"), f"{prefix}.position.lat")
        longitude_deg = _finite_number(position.get("lon"), f"{prefix}.position.lon")
        if not -90.0 <= latitude_deg <= 90.0:
            raise TrafficProtocolError(f"{prefix}.position.lat is out of range")
        if not -180.0 <= longitude_deg <= 180.0:
            raise TrafficProtocolError(f"{prefix}.position.lon is out of range")

        speed_mps = _finite_number(position.get("speed"), f"{prefix}.position.speed")
        if speed_mps < 0.0:
            raise TrafficProtocolError(f"{prefix}.position.speed must be non-negative")
        heading_deg_true = _finite_number(
            position.get("direction"), f"{prefix}.position.direction"
        )
        if not 0.0 <= heading_deg_true < 360.0:
            raise TrafficProtocolError(
                f"{prefix}.position.direction must be in [0, 360)"
            )

        parsed.append(
            TrafficAircraftData(
                aircraft_index=aircraft_index,
                name=name,
                latitude_deg=latitude_deg,
                longitude_deg=longitude_deg,
                altitude_agl_m=_finite_number(
                    position.get("altitude"), f"{prefix}.position.altitude"
                ),
                speed_mps=speed_mps,
                heading_deg_true=heading_deg_true,
                horizontal_keepaway_m=_positive_number(
                    entry.get("horizontalKeepAway"),
                    f"{prefix}.horizontalKeepAway",
                ),
                vertical_keepaway_m=_positive_number(
                    entry.get("verticalKeepAway"),
                    f"{prefix}.verticalKeepAway",
                ),
            )
        )

    return TrafficEvent(tuple(parsed))
