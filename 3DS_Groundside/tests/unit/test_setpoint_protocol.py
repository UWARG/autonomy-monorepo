"""Unit tests for src.setpoint_tester.protocol."""

import json
import math

import pytest

from src.setpoint_tester.protocol import (
    Reply,
    encode,
    go_home,
    parse_reply,
    setpoint,
)


def test_setpoint_command() -> None:
    assert setpoint(43.4339, -80.5776, 10.0) == {
        "type": "setpoint",
        "lat": 43.4339,
        "lon": -80.5776,
        "alt": 10.0,
    }


def test_setpoint_converts_integers_to_floats() -> None:
    command = setpoint(43, -80, 10)
    assert all(isinstance(command[key], float) for key in ("lat", "lon", "alt"))


def test_setpoint_leaves_min_altitude_to_the_drone() -> None:
    # The drone owns the altitude limit and replies with an error.
    assert setpoint(43.4, -80.5, 0.5)["alt"] == 0.5


@pytest.mark.parametrize(
    "lat, lon, alt",
    [
        (90.1, 0.0, 10.0),
        (-90.1, 0.0, 10.0),
        (0.0, 180.1, 10.0),
        (0.0, -180.1, 10.0),
        (math.nan, 0.0, 10.0),
        (0.0, math.inf, 10.0),
        (0.0, 0.0, math.nan),
        (0.0, 0.0, -math.inf),
        (True, 0.0, 10.0),
        ("43.4", 0.0, 10.0),
    ],
)
def test_setpoint_rejects_invalid(lat: object, lon: object, alt: object) -> None:
    with pytest.raises((ValueError, TypeError)):
        setpoint(lat, lon, alt)  # type: ignore[arg-type]


def test_go_home_command() -> None:
    assert go_home() == {"type": "go_home"}


def test_encode_is_one_json_line() -> None:
    line = encode(setpoint(43.4339, -80.5776, 10.0))

    assert line.endswith(b"\n")
    assert line.count(b"\n") == 1
    assert json.loads(line) == setpoint(43.4339, -80.5776, 10.0)


def test_parse_reply() -> None:
    assert parse_reply(b'{"ok": true, "message": "published"}\n') == Reply(
        ok=True, message="published"
    )
    assert parse_reply(b'{"ok": false, "message": "rejected: x"}\n') == Reply(
        ok=False, message="rejected: x"
    )


@pytest.mark.parametrize(
    "line",
    [
        b"",
        b"nope\n",
        b"\xff\xfe\n",
        b"[]\n",
        b'{"ok": true}\n',
        b'{"message": "x"}\n',
        b'{"ok": "yes", "message": "x"}\n',
        b'{"ok": true, "message": 3}\n',
    ],
)
def test_parse_reply_rejects_malformed(line: bytes) -> None:
    with pytest.raises((ValueError, TypeError)):
        parse_reply(line)
