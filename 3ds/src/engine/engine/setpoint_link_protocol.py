"""
Wire protocol of the setpoint tester link (no ROS dependencies).

The ground sends one JSON object per line (UTF-8, ``\\n`` terminated):

    {"type": "setpoint", "lat": 43.4339, "lon": -80.5776, "alt": 10.0}
    {"type": "go_home"}

and the airside answers every non-blank line with one JSON reply line:

    {"ok": true, "message": "..."}

The groundside client lives in ``3DS_Groundside/src/setpoint_tester``; keep the
two in sync.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Union

from engine.constants import SETPOINT_MIN_ALTITUDE_M
from utils.src.types import Coordinate

DEFAULT_PORT = 7070

# Longest accepted line, newline included. Real commands are under 100 bytes.
MAX_LINE_BYTES = 1024

SETPOINT_TYPE = "setpoint"
GO_HOME_TYPE = "go_home"

_SETPOINT_FIELDS = {"type", "lat", "lon", "alt"}
_GO_HOME_FIELDS = {"type"}


@dataclass(frozen=True)
class GoHome:
    """Command to return to the launch point and land."""


Command = Union[Coordinate, GoHome]


def _number(message: dict[str, Any], field: str) -> float:
    value = message[field]
    # bool is a subclass of int, but `true` is not a coordinate
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"'{field}' must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"'{field}' must be finite, got {value!r}")
    return float(value)


def _check_fields(message: dict[str, Any], expected: set[str]) -> None:
    missing = expected - message.keys()
    if missing:
        raise ValueError(f"missing field(s): {', '.join(sorted(missing))}")
    unknown = message.keys() - expected
    if unknown:
        raise ValueError(f"unknown field(s): {', '.join(sorted(unknown))}")


def parse_command(line: bytes) -> Command:
    """
    Parses one received line into a command.

    Raises ``ValueError`` or ``TypeError`` with an operator-readable reason if
    the line is not valid JSON, is not a known command, or is a setpoint the
    engine would reject (lat/lon out of range, altitude below
    ``SETPOINT_MIN_ALTITUDE_M``).
    """

    try:
        message = json.loads(line)
    except ValueError as error:  # also covers invalid UTF-8
        raise ValueError(f"not valid JSON: {error}") from None
    if not isinstance(message, dict):
        raise TypeError("expected a JSON object")

    command_type = message.get("type")
    if command_type == GO_HOME_TYPE:
        _check_fields(message, _GO_HOME_FIELDS)
        return GoHome()
    if command_type != SETPOINT_TYPE:
        raise ValueError(
            f"unknown command type {command_type!r} "
            f"(expected '{SETPOINT_TYPE}' or '{GO_HOME_TYPE}')"
        )

    _check_fields(message, _SETPOINT_FIELDS)
    lat = _number(message, "lat")
    lon = _number(message, "lon")
    alt = _number(message, "alt")
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"lat {lat} is outside [-90, 90]")
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"lon {lon} is outside [-180, 180]")
    if alt < SETPOINT_MIN_ALTITUDE_M:
        raise ValueError(f"alt {alt}m is below the {SETPOINT_MIN_ALTITUDE_M}m minimum")
    return Coordinate(lat=lat, lon=lon, alt=alt)


def encode_reply(ok: bool, message: str) -> bytes:
    """Encodes one reply line."""

    return (json.dumps({"ok": ok, "message": message}) + "\n").encode("utf-8")
