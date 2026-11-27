"""Sender #230: local tests; synthetic delays do not measure AEAC arrival timing.

Strict xfails document sender defects; --runxfail exposes their failures.
"""

import json
import logging
import runpy
import socket
import time
from collections import deque
from contextlib import contextmanager, suppress
from itertools import pairwise, product
from threading import Event, Thread
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock
from urllib.parse import parse_qs, urlsplit

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from test_vehicle import flying_vehicle
from websockets.exceptions import (
    ConnectionClosed,
    ConnectionClosedError,
    InvalidHandshake,
)
from websockets.sync.client import connect
from websockets.sync.server import serve

from aeac_telemetry import main as sender
from aeac_telemetry.config import DEFAULT_MAVLINK_URL, DEFAULT_WEBSOCKET_URL, Config

TOKEN, UAV = "AEAC_CONNECTION_TOKEN", "AEAC_UAV_ID"
CONFIG = Config("udpin:127.0.0.1:14551", "ws://127.0.0.1:8765", "a /?&+é", "TEST-230")


def messages(kind=mavlink.MAV_TYPE_QUADROTOR):
    vehicle = flying_vehicle()
    heartbeat = vehicle.heartbeat
    heartbeat.type = kind
    if kind != mavlink.MAV_TYPE_QUADROTOR:
        heartbeat.autopilot = mavlink.MAV_AUTOPILOT_INVALID
    heartbeat._header.srcSystem = 17
    heartbeat._header.srcComponent = 1
    vehicle.position.relative_alt = 15234
    return [
        heartbeat,
        vehicle.position,
        vehicle.gps,
        mavlink.MAVLink_sys_status_message(0, 0, 0, 0, 0, 0, 87, 0, 0, 0, 0, 0, 0),
        mavlink.MAVLink_system_time_message(1_800_000_000_000_000, 0),
        mavlink.MAVLink_rc_channels_message(0, 8, *([1500] * 18), 127),
    ]


def known_defect(reason):
    return pytest.mark.xfail(strict=True, reason=f"#230: {reason}")


def gaps(timestamps):
    return [round(right - left, 9) for left, right in pairwise(timestamps)]


def assert_contract(timestamps):
    intervals = gaps(timestamps)
    # Rounding removes floating representation error, not scheduler delay.
    assert intervals and all(0.4 <= gap <= 1.1 for gap in intervals), intervals


class Harness:
    """One controlled clock and two external connections; real sender/vehicle/messages."""

    def __init__(self, patch, stop=2.2):
        self.now, self.stop, self.calls = 0.0, stop, 0
        self.input, self.receive_delays, self.send_delays = messages(), (0.05,), (0.0,)
        self.starts, self.completions, self.packets, self.sleeps = [], [], [], []
        self.replies, self.failures = deque(), {}
        self.flood, self.grid, self.stall, self.reply_cost = False, False, False, 0.0
        self.mav = Mock(recv_match=self.read)
        self.ws = MagicMock(send=self.send, recv=self.recv)
        self.ws.__enter__.return_value = self.ws
        self.mav_open, self.connect = (
            Mock(return_value=self.mav),
            Mock(return_value=self.ws),
        )
        patch.setattr(sender, "time", self)
        patch.setattr(sender.mavutil, "mavlink_connection", self.mav_open)
        patch.setattr(sender, "connect", self.connect)

    def monotonic(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds
        if self.now >= self.stop:
            raise KeyboardInterrupt

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.advance(seconds)

    def fail(self, where):
        failure = self.failures.pop(where, None)
        if failure is not None:
            raise failure

    def read(self, *, blocking, timeout):
        assert (blocking, timeout) == (True, 0.05)
        self.calls += 1
        if self.stall and self.calls == 30:
            self.advance(3.0)
        delay = (
            (int(self.now * 32) + 1) / 32 - self.now
            if self.grid
            else self.receive_delays[(self.calls - 1) % len(self.receive_delays)]
        )
        self.advance(delay)
        return self.input[(self.calls - 1) % len(self.input)] if self.input else None

    def send(self, payload):
        self.fail("send")
        self.starts.append(self.now)
        self.packets.append(json.loads(payload))
        self.advance(self.send_delays[(len(self.starts) - 1) % len(self.send_delays)])
        self.completions.append(self.now)

    def recv(self, *, timeout):
        self.fail("recv")
        assert timeout == 0
        if not self.replies and not self.flood:
            raise TimeoutError
        self.advance(self.reply_cost)
        return self.replies.popleft() if self.replies else '{"event":"traffic"}'

    def run(self):
        with pytest.raises(KeyboardInterrupt):
            sender.run(CONFIG)


@pytest.mark.parametrize(
    "case,blank",
    list(product((TOKEN, UAV, "both"), (None, "", " \t ")))
    + [("defaults", value) for value in (None, "", "  ")]
    + [("explicit", None)],
)
def test_environment(monkeypatch, case, blank):
    values = {
        "MAVLINK_URL": f" {CONFIG.mavlink_url} ",
        "AEAC_WEBSOCKET_URL": f" {CONFIG.websocket_url} ",
        TOKEN: f" {CONFIG.token} ",
        UAV: f" {CONFIG.uav_id} ",
    }
    missing = [TOKEN, UAV] if case == "both" else [case]
    changed = ["MAVLINK_URL", "AEAC_WEBSOCKET_URL"] if case == "defaults" else missing
    if case != "explicit":
        values.update(dict.fromkeys(changed, blank))
    for name, value in values.items():
        monkeypatch.delenv(name, raising=False)
        if value is not None:
            monkeypatch.setenv(name, value)
    if case in (TOKEN, UAV, "both"):
        with pytest.raises(ValueError) as error:
            Config.from_env()
        assert str(error.value) == "missing environment variables: " + ", ".join(
            missing
        )
        assert CONFIG.token not in str(error.value)
    else:
        expected = (
            Config(
                DEFAULT_MAVLINK_URL, DEFAULT_WEBSOCKET_URL, CONFIG.token, CONFIG.uav_id
            )
            if case == "defaults"
            else CONFIG
        )
        assert Config.from_env() == expected


@pytest.mark.parametrize(
    "reply,expected",
    [
        ({"event": "telemetry_ack"}, []),
        ({"event": "telemetry_ack", "insideBoundary": True}, []),
        ({"event": "telemetry_ack", "insideBoundary": 0}, []),
        (
            {"event": "telemetry_ack", "insideBoundary": False},
            ["server: outside the flight boundary"],
        ),
        (
            {"event": "telemetry_ack", "tooCloseToTraffic": True},
            ["server: too close to traffic"],
        ),
        (
            {
                "event": "telemetry_ack",
                "insideBoundary": False,
                "tooCloseToTraffic": True,
            },
            ["server: outside the flight boundary", "server: too close to traffic"],
        ),
        ({"event": "infraction", "last_infraction": "timing"}, ["infraction: timing"]),
        (
            {"event": "error", "message": "synthetic rejection"},
            ["server error: synthetic rejection"],
        ),
        ({"event": "traffic"}, []),
        ({"event": "future_event"}, []),
        ({}, []),
    ],
)
def test_server_events(caplog, reply, expected):
    sender._handle_event(reply)
    level = logging.ERROR if reply.get("event") == "error" else logging.WARNING
    assert [(record.levelno, record.message) for record in caplog.records] == [
        (level, text) for text in expected
    ]


def test_connection_arguments_and_packet_calculations(monkeypatch):
    h = Harness(monkeypatch)
    h.input += [h.input[0]] * 100  # SYSTEM_TIME arrives once, at t=0.25.
    h.run()
    h.mav_open.assert_called_once_with(CONFIG.mavlink_url, source_system=254)
    h.connect.assert_called_once_with(
        "ws://127.0.0.1:8765?Authorization=a%20%2F%3F%26%2B%C3%A9", open_timeout=10
    )
    h.mav.mav.request_data_stream_send.assert_called_once_with(
        17, 1, mavlink.MAV_DATA_STREAM_ALL, 4, 1
    )
    assert h.ws.__exit__.called
    assert h.packets[0] == {
        "action": "telemetry",
        "data": {
            "uavId": CONFIG.uav_id,
            "unixTime": pytest.approx(
                1_800_000_000 + h.starts[0] - 0.25, rel=0, abs=0.001
            ),
            "latitude": pytest.approx(50.1022871),
            "longitude": pytest.approx(-110.7373276),
            "altitudeAGL": 15.23,
            "horizontalPositionAccuracy": 0.8,
            "verticalPositionAccuracy": 1.5,
            "batteryPercentage": 87.0,
            "mode": "armed-automatic",
            "telemetryLinkStatus": 1.0,
            "rcLinkStatus": 0.5,
        },
    }


@pytest.mark.parametrize(
    "case",
    ["waiting", "no-fix", "malformed"]
    + [
        pytest.param(reply, marks=known_defect("non-object reply crashes"))
        for reply in ("null", "[]", "1", '"text"')
    ],
)
def test_sender_inputs(monkeypatch, caplog, case):
    h = Harness(monkeypatch, stop=3.2)
    caplog.set_level(logging.INFO)
    if case in ("waiting", "no-fix"):
        h.input = [] if case == "waiting" else h.input[:1]
    else:
        h.replies.extend(
            ["{", '{"event":"infraction","last_infraction":"test"}']
            if case == "malformed"
            else [case]
        )
    h.run()
    if case == "waiting":
        assert h.packets == []
        assert f"waiting for a heartbeat on {CONFIG.mavlink_url}" in caplog.text
    else:
        assert len(h.packets) >= 2
        if case == "no-fix":
            assert h.packets[0]["data"]["latitude"] == 0
            assert h.packets[0]["data"]["horizontalPositionAccuracy"] == 9999
            assert "no GPS fix (armed-automatic)" in caplog.text
        elif case == "malformed":
            assert "unparseable server message" in caplog.text
            assert "infraction: test" in caplog.text


@pytest.mark.parametrize(
    "where,failure",
    list(
        product(
            ("connect", "recv", "send"),
            (OSError("offline"), ConnectionClosedError(None, None)),
        )
    )
    + [
        ("handshake", InvalidHandshake("denied")),
        ("mav", OSError("MAVLink unavailable")),
    ],
)
def test_connection_failures(monkeypatch, caplog, where, failure):
    h = Harness(monkeypatch, stop=4.2)
    if where in ("connect", "handshake"):
        h.connect.side_effect = [failure, h.ws]
    elif where == "mav":
        h.mav_open.side_effect = failure
    else:
        h.failures[where] = failure
    if where in ("handshake", "mav"):
        error, match = (
            (SystemExit, "Server rejected.*check the token")
            if where == "handshake"
            else (OSError, "MAVLink unavailable")
        )
        with pytest.raises(error, match=match):
            sender.run(CONFIG)
        assert h.sleeps == [] and h.packets == []
        assert h.connect.call_count == int(where == "handshake")
    else:
        h.run()
        assert h.sleeps == [1.0] and h.connect.call_count == 2
        assert h.packets and h.ws.__exit__.called
        assert "retrying" in caplog.text
        h.mav.mav.request_data_stream_send.assert_called_once()


@pytest.mark.parametrize(
    "kind",
    [mavlink.MAV_TYPE_QUADROTOR]
    + [
        pytest.param(kind, marks=known_defect("peripheral requests streams"))
        for kind in (mavlink.MAV_TYPE_GCS, mavlink.MAV_TYPE_ONBOARD_CONTROLLER)
    ],
)
def test_stream_request_uses_first_aircraft_heartbeat(monkeypatch, kind):
    h = Harness(monkeypatch)
    h.input = messages(kind)[:1]
    h.run()
    accepted = kind == mavlink.MAV_TYPE_QUADROTOR
    assert h.mav.mav.request_data_stream_send.call_count == int(accepted)
    assert bool(h.packets) == accepted


@pytest.mark.parametrize("case", ["bad-config", "normal", "interrupt", "module"])
def test_entrypoints(monkeypatch, caplog, case):
    invalid = case in ("bad-config", "module")
    monkeypatch.setattr(
        Config,
        "from_env",
        Mock(
            return_value=CONFIG,
            side_effect=ValueError("missing test") if invalid else None,
        ),
    )
    run = Mock(side_effect=KeyboardInterrupt if case == "interrupt" else None)
    monkeypatch.setattr(sender, "run", run)
    if case == "module":
        with (
            pytest.warns(RuntimeWarning, match="found in sys.modules"),
            pytest.raises(SystemExit) as error,
        ):
            runpy.run_module("aeac_telemetry.main", run_name="__main__")
        assert error.value.code == 1
    else:
        assert sender.main() == int(invalid)
    if invalid:
        assert "missing test" in caplog.text
        run.assert_not_called()
    else:
        run.assert_called_once_with(CONFIG)


@pytest.mark.parametrize(
    "receive,send",
    [
        ((0.05,), (0.0,)),
        ((0.001,), (0.0,)),
        ((0.002, 0.049, 0.017, 0.031), (0.0,)),
        ((0.05,), (0.02, 0.08, 0.04)),
    ],
    ids=["ordinary", "busy-mavlink", "receive-jitter", "synthetic-send-delays"],
)
def test_sustained_cadence(monkeypatch, receive, send):
    h = Harness(monkeypatch, stop=61.2)
    h.receive_delays, h.send_delays = receive, send
    h.run()
    assert len(h.starts) >= 55
    assert_contract(h.starts)
    assert_contract(h.completions)


@pytest.mark.parametrize(
    "continuous",
    [False, pytest.param(True, marks=known_defect("server flood starves sender"))],
)
def test_server_flood(monkeypatch, continuous):
    h = Harness(monkeypatch, stop=10.0 if continuous else 31.2)
    h.flood, h.reply_cost = continuous, 0.01 if continuous else 0.0001
    h.replies.extend([] if continuous else ['{"event":"traffic"}'] * 500)
    h.run()
    assert not h.replies
    assert len(h.starts) >= (8 if continuous else 28)
    assert_contract(h.starts)
    assert_contract(h.completions)


@pytest.mark.parametrize(
    "delays,bound",
    [
        ((0.6, 0.0), "lower"),
        ((0.0, 0.1), "upper"),
        pytest.param(
            (0.600001, 0.0), "lower", marks=known_defect("bunched completions")
        ),
        pytest.param((0.0, 0.100001), "upper", marks=known_defect("late completions")),
    ],
)
def test_synthetic_send_completion_boundaries(monkeypatch, delays, bound):
    h = Harness(monkeypatch, stop=2.5)
    # Arrivals align to a binary-exact grid including deadlines: no polling overshoot.
    h.grid, h.send_delays = True, delays
    h.run()
    assert_contract(h.starts)
    gap = gaps(h.completions)[0]
    assert gap >= 0.4 if bound == "lower" else gap <= 1.1


@pytest.mark.parametrize(
    "stall",
    [
        "receive",
        "send",
        pytest.param(
            "completion", marks=known_defect("blocked send bunches completions")
        ),
    ],
)
def test_synthetic_stall_and_recovery(monkeypatch, stall):
    h = Harness(monkeypatch, stop=12.0 if stall == "completion" else 20.0)
    h.stall = stall == "receive"
    if stall != "receive":
        h.send_delays = (0.0, 0.0, 3.0) + (0.0,) * (3 if stall == "completion" else 30)
    h.run()
    assert any(
        gap > 1.1 for gap in gaps(h.starts)
    )  # Impossible upper bound during stall.
    assert all(gap >= 0.4 for gap in gaps(h.starts))
    if stall == "completion":
        assert all(gap >= 0.4 for gap in gaps(h.completions))
    else:
        assert_contract(h.starts[-10:])
        assert_contract(h.completions[-10:])


@pytest.mark.parametrize("disconnect", [False, True], ids=["sustained", "reconnect"])
def test_real_udp_websocket(monkeypatch, disconnect):
    stop, ready, done = Event(), Event(), Event()
    starts, completions, receipts, packets, paths, errors = [], [], [], [], [], []
    mav = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=254)
    port, original_receive = mav.port.getsockname()[1], mav.recv_match

    def receive(**kwargs):
        if stop.is_set():
            raise KeyboardInterrupt
        return original_receive(**kwargs)

    @contextmanager
    def recording_connection(url, **kwargs):
        if stop.is_set():
            raise KeyboardInterrupt
        with connect(url, **kwargs) as ws:

            def send(payload):
                starts.append(time.monotonic())
                ws.send(payload)
                completions.append(time.monotonic())

            yield SimpleNamespace(send=send, recv=ws.recv)

    def handle(ws):
        received = []
        receipts.append(received)
        paths.append(ws.request.path)
        try:
            for payload in ws:
                received.append(time.monotonic())
                packets.append(json.loads(payload))
                if disconnect and len(paths) == 1 and len(packets) == 4:
                    ws.close(code=1012, reason="synthetic local restart")
                    return
                ws.send('{"event":"telemetry_ack","insideBoundary":true}')
                if len(packets) >= 12:
                    done.set()
        except ConnectionClosed:
            pass
        except Exception as error:  # noqa: BLE001 - propagate server errors to pytest
            errors.append(error)
            done.set()

    def feed():
        encoder = mavlink.MAVLink(None, srcSystem=17, srcComponent=1)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            while not stop.is_set():
                for message in messages():
                    udp.sendto(message.pack(encoder), ("127.0.0.1", port))
                    encoder.seq = (encoder.seq + 1) % 256
                ready.set()
                stop.wait(0.1)

    with serve(handle, "127.0.0.1", 0) as server:
        config = Config(
            f"udpin:127.0.0.1:{port}",
            f"ws://127.0.0.1:{server.socket.getsockname()[1]}",
            "local /?&+é",
            "LOCAL-230",
        )
        monkeypatch.setattr(mav, "recv_match", receive)
        monkeypatch.setattr(
            sender.mavutil, "mavlink_connection", Mock(return_value=mav)
        )
        monkeypatch.setattr(sender, "connect", recording_connection)

        def run():
            with suppress(KeyboardInterrupt):
                sender.run(config)

        def worker(target):
            try:
                target()
            except Exception as error:  # noqa: BLE001 - report thread failures in pytest
                errors.append(error)
            finally:
                done.set()

        workers = [
            Thread(target=worker, args=(target,), daemon=True)
            for target in (server.serve_forever, feed, run)
        ]
        try:
            workers[0].start()
            workers[1].start()
            assert ready.wait(2), "local MAVLink feeder did not start"
            workers[2].start()
            assert done.wait(25), "local sender did not deliver twelve packets"
        finally:
            stop.set()
            for thread in workers[1:]:
                if thread.ident is not None:
                    thread.join(timeout=3)
            server.shutdown()
            workers[0].join(timeout=3)
            mav.close()
        assert not any(thread.is_alive() for thread in workers)
    assert errors == [] and len(packets) >= 12
    assert len(paths) == 1 + disconnect
    assert all(
        parse_qs(urlsplit(path).query) == {"Authorization": [config.token]}
        for path in paths
    )
    assert all(packet["action"] == "telemetry" for packet in packets)
    assert packets[0]["data"]["uavId"] == "LOCAL-230"
    assert packets[0]["data"]["latitude"] == pytest.approx(50.1022871)
    assert packets[0]["data"]["altitudeAGL"] == 15.23
    for received in receipts:
        assert_contract(received)
    if disconnect:
        # Outage is measured separately; its retry delay can exceed the upper bound.
        assert receipts[1][0] - receipts[0][-1] >= 1.0
    assert_contract(starts[-8:] if disconnect else starts)
    assert_contract(completions[-8:] if disconnect else completions)
