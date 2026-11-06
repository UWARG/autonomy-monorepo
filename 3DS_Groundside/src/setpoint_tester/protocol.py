"""Wire protocol of the setpoint tester link.

The ground sends one JSON object per line (UTF-8, newline terminated) and the
drone answers every non-blank line with one JSON reply line
``{"ok": bool, "message": str}``.
The airside end is ``3ds/src/engine/engine/setpoint_link_protocol.py``; keep the
two in sync.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

DEFAULT_PORT = 7070


@dataclass(frozen=True)
class Reply:
    """The drone's answer to one command."""

    ok: bool
    message: str


def _coordinate(name: str, value: float, limit: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    if abs(value) > limit:
        raise ValueError(f"{name} must be within ±{limit:g}, got {value}")
    return float(value)


def setpoint(lat: float, lon: float, alt: float) -> dict[str, Any]:
    """Build a setpoint command: fly to (lat, lon) at ``alt`` meters above launch
    and take a photo.

    Raises ValueError for coordinates that can never be valid and TypeError for
    non-numbers. The drone enforces its own minimum altitude and replies with an
    error if ``alt`` is too low.
    """
    return {
        "type": "setpoint",
        "lat": _coordinate("lat", lat, 90.0),
        "lon": _coordinate("lon", lon, 180.0),
        "alt": _coordinate("alt", alt, math.inf),
    }


def go_home() -> dict[str, Any]:
    """Build the go-home command: return to the launch point and land."""
    return {"type": "go_home"}


def encode(command: dict[str, Any]) -> bytes:
    """One command as a newline-terminated line."""
    return (json.dumps(command, allow_nan=False) + "\n").encode("utf-8")


def parse_reply(line: bytes) -> Reply:
    """Parse one reply line.

    Raises ValueError if it is not JSON and TypeError if it is JSON of the wrong
    shape.
    """
    try:
        reply = json.loads(line)
    except ValueError as error:  # also covers invalid UTF-8
        raise ValueError(f"reply is not valid JSON: {line!r}") from error
    if (
        not isinstance(reply, dict)
        or not isinstance(reply.get("ok"), bool)
        or not isinstance(reply.get("message"), str)
    ):
        raise TypeError(f"malformed reply: {line!r}")
    return Reply(ok=reply["ok"], message=reply["message"])
