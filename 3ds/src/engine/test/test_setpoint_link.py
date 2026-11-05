import json
import socket
import time

import pytest
import rclpy
from airside_interfaces.msg import Coordinate as CoordinateMsg
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Empty

from engine.constants import SETPOINT_MIN_ALTITUDE_M
from engine.setpoint_link_node import SetpointLink
from engine.setpoint_link_protocol import (
    MAX_LINE_BYTES,
    GoHome,
    encode_reply,
    parse_command,
)
from utils.src.types import Coordinate

TIMEOUT_S = 5.0


# --- protocol -----------------------------------------------------------------


def test_parse_setpoint():
    line = b'{"type": "setpoint", "lat": 43.4339, "lon": -80.5776, "alt": 10.0}\n'
    assert parse_command(line) == Coordinate(lat=43.4339, lon=-80.5776, alt=10.0)


def test_parse_setpoint_accepts_integers():
    command = parse_command(b'{"type": "setpoint", "lat": 43, "lon": -80, "alt": 10}')
    assert command == Coordinate(lat=43.0, lon=-80.0, alt=10.0)
    assert isinstance(command.alt, float)


def test_parse_setpoint_at_min_altitude():
    line = json.dumps({"type": "setpoint", "lat": 0, "lon": 0, "alt": SETPOINT_MIN_ALTITUDE_M})
    assert parse_command(line.encode()).alt == SETPOINT_MIN_ALTITUDE_M


def test_parse_go_home():
    assert parse_command(b'{"type": "go_home"}\n') == GoHome()


@pytest.mark.parametrize(
    "line",
    [
        b"",
        b"not json",
        b"\xff\xfe",
        b"[1, 2, 3]",
        b'"setpoint"',
        b"{}",
        b'{"type": "takeoff"}',
        b'{"type": "go_home", "now": true}',
        # missing / unknown fields
        b'{"type": "setpoint", "lat": 43.4, "lon": -80.5}',
        b'{"type": "setpoint", "lat": 43.4, "lon": -80.5, "altitude": 10}',
        b'{"type": "setpoint", "lat": 43.4, "lon": -80.5, "alt": 10, "yaw": 0}',
        # wrong types
        b'{"type": "setpoint", "lat": "43.4", "lon": -80.5, "alt": 10}',
        b'{"type": "setpoint", "lat": true, "lon": -80.5, "alt": 10}',
        b'{"type": "setpoint", "lat": null, "lon": -80.5, "alt": 10}',
        # non-finite (Python's json accepts these tokens)
        b'{"type": "setpoint", "lat": NaN, "lon": -80.5, "alt": 10}',
        b'{"type": "setpoint", "lat": 43.4, "lon": -80.5, "alt": Infinity}',
        # out of range
        b'{"type": "setpoint", "lat": 90.1, "lon": -80.5, "alt": 10}',
        b'{"type": "setpoint", "lat": -90.1, "lon": -80.5, "alt": 10}',
        b'{"type": "setpoint", "lat": 43.4, "lon": 180.1, "alt": 10}',
        b'{"type": "setpoint", "lat": 43.4, "lon": -180.1, "alt": 10}',
        b'{"type": "setpoint", "lat": 43.4, "lon": -80.5, "alt": 1.9}',
        b'{"type": "setpoint", "lat": 43.4, "lon": -80.5, "alt": -5}',
    ],
)
def test_parse_rejects_invalid(line):
    with pytest.raises((ValueError, TypeError)):
        parse_command(line)


def test_encode_reply():
    line = encode_reply(False, "rejected: nope")
    assert line.endswith(b"\n")
    assert json.loads(line) == {"ok": False, "message": "rejected: nope"}


# --- node ---------------------------------------------------------------------


@pytest.fixture
def link():
    rclpy.init()
    node = SetpointLink(
        parameter_overrides=[
            Parameter("host", value="127.0.0.1"),
            Parameter("port", value=0),
        ]
    )
    yield node
    node.close()
    node.destroy_node()
    rclpy.try_shutdown()


class _Listener:
    """Records what reaches the engine's command topics."""

    def __init__(self, link):
        self.setpoints = []
        self.go_homes = []
        self.node = rclpy.create_node("test_listener")
        self.node.create_subscription(
            CoordinateMsg, "mission/setpoint", self.setpoints.append, 10
        )
        self.node.create_subscription(Empty, "mission/go_home", self.go_homes.append, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Volatile topics drop messages published before discovery completes.
        _wait_until(
            lambda: link._setpoint_pub.get_subscription_count() > 0
            and link._go_home_pub.get_subscription_count() > 0
        )

    def spin_until(self, condition):
        deadline = time.monotonic() + TIMEOUT_S
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        return condition()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()


@pytest.fixture
def listener(link):
    listener = _Listener(link)
    yield listener
    listener.close()


def _wait_until(condition):
    deadline = time.monotonic() + TIMEOUT_S
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.05)


class _Ground:
    """A minimal ground client speaking the line protocol."""

    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=TIMEOUT_S)
        self.reader = self.sock.makefile("rb")

    def send(self, line: bytes) -> dict:
        self.sock.sendall(line)
        return json.loads(self.reader.readline())

    def close(self):
        self.reader.close()
        self.sock.close()


@pytest.fixture
def ground(link):
    ground = _Ground(link.port)
    yield ground
    ground.close()


def test_forwards_setpoint(link, listener, ground):
    reply = ground.send(b'{"type": "setpoint", "lat": 43.4339, "lon": -80.5776, "alt": 10}\n')

    assert reply["ok"] is True
    assert "/mission/setpoint" in reply["message"]
    assert "WARNING" not in reply["message"]
    assert listener.spin_until(lambda: listener.setpoints)
    msg = listener.setpoints[0]
    assert (msg.lat, msg.lon, msg.alt) == (43.4339, -80.5776, 10.0)
    assert listener.go_homes == []


def test_forwards_go_home(link, listener, ground):
    reply = ground.send(b'{"type": "go_home"}\n')

    assert reply["ok"] is True
    assert "/mission/go_home" in reply["message"]
    assert listener.spin_until(lambda: listener.go_homes)
    assert listener.setpoints == []


def test_forwards_several_commands_in_order(link, listener, ground):
    for lat in (43.1, 43.2, 43.3):
        line = json.dumps({"type": "setpoint", "lat": lat, "lon": -80.5, "alt": 10})
        assert ground.send(line.encode() + b"\n")["ok"] is True

    assert listener.spin_until(lambda: len(listener.setpoints) == 3)
    assert [msg.lat for msg in listener.setpoints] == [43.1, 43.2, 43.3]


def test_rejects_invalid_and_keeps_connection(link, listener, ground):
    reply = ground.send(b'{"type": "setpoint", "lat": 43.4, "lon": -80.5, "alt": 0.5}\n')
    assert reply["ok"] is False
    assert "minimum" in reply["message"]

    reply = ground.send(b'{"type": "setpoint", "lat": "43.4", "lon": -80.5, "alt": 10}\n')
    assert reply["ok"] is False
    assert "must be a number" in reply["message"]

    reply = ground.send(b"[1, 2]\n")
    assert reply["ok"] is False
    assert "JSON object" in reply["message"]

    # Blank lines are ignored; the connection still serves the next command.
    reply = ground.send(b'\n{"type": "go_home"}\n')
    assert reply["ok"] is True
    assert listener.spin_until(lambda: listener.go_homes)
    assert listener.setpoints == []


def test_rejects_overlong_line_and_keeps_connection(link, listener, ground):
    reply = ground.send(b"x" * (3 * MAX_LINE_BYTES + 10) + b"\n")
    assert reply["ok"] is False
    assert "longer than" in reply["message"]

    assert ground.send(b'{"type": "go_home"}\n')["ok"] is True
    assert listener.spin_until(lambda: listener.go_homes)


def test_warns_when_engine_is_not_listening(link, ground):
    reply = ground.send(b'{"type": "go_home"}\n')

    assert reply["ok"] is True
    assert "nothing is subscribed" in reply["message"]


def test_serves_reconnecting_ground(link, listener, ground):
    ground.close()

    second = _Ground(link.port)
    try:
        assert second.send(b'{"type": "go_home"}\n')["ok"] is True
    finally:
        second.close()
    assert listener.spin_until(lambda: listener.go_homes)
