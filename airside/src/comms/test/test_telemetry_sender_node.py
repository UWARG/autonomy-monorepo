from __future__ import annotations

import json
import time

import pytest
import rclpy
import websocket
from comms.constants import CONNECTION_TOKEN_ENV, UAV_ID_ENV
from comms.telemetry_sender_node import TelemetrySender
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float64


@pytest.fixture
def sender(monkeypatch: pytest.MonkeyPatch) -> TelemetrySender:
    monkeypatch.setenv(CONNECTION_TOKEN_ENV, "test-token")
    monkeypatch.setenv(UAV_ID_ENV, "WARG-01")
    rclpy.init()
    node = TelemetrySender()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _ready_inputs(sender: TelemetrySender) -> None:
    fix = NavSatFix(latitude=43.47, longitude=-80.54)
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    fix.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0]
    fix.header.stamp = sender.get_clock().now().to_msg()
    sender._fix_callback(fix)
    sender._altitude_callback(Float64(data=15.0))
    sender._state_callback(State(connected=True, armed=True, mode="GUIDED"))
    sender._battery_callback(BatteryState(percentage=0.82))


def test_sender_drains_traffic_and_reaches_ack(
    sender: TelemetrySender, monkeypatch: pytest.MonkeyPatch
) -> None:
    class FakeSocket:
        def __init__(self) -> None:
            self.sent: list[str] = []
            self.events = [
                json.dumps({"event": "traffic", "payload": {"traffic": []}}),
                json.dumps({"event": "telemetry_ack"}),
            ]

        def settimeout(self, timeout: float) -> None:
            assert timeout <= 0.1

        def send(self, payload: str) -> None:
            self.sent.append(payload)

        def recv(self) -> str:
            if not self.events:
                raise websocket.WebSocketTimeoutException()
            return self.events.pop(0)

        def close(self) -> None:
            pass

    socket = FakeSocket()
    diagnostics: list[tuple[str, bool]] = []
    monkeypatch.setattr(websocket, "create_connection", lambda *args, **kwargs: socket)
    monkeypatch.setattr(
        sender,
        "_publish_diagnostic",
        lambda reason, *, ready, interval_s=None: diagnostics.append((reason, ready)),
    )
    _ready_inputs(sender)

    sender._tick()

    assert sender._sent_count == 1
    assert sender._last_ack_s is not None
    assert diagnostics[-1] == ("AEAC_ACK_RECENT", True)
    assert json.loads(socket.sent[0])["data"]["altitudeAGL"] == 15.0


def test_sender_rejects_old_fix_even_if_callback_was_recent(sender: TelemetrySender) -> None:
    _ready_inputs(sender)
    assert sender._fix is not None
    sender._fix.header.stamp.sec = int(time.time()) - 5

    payload, reason = sender._payload()

    assert payload is None
    assert reason == "STALE_GPS_MEASUREMENT"


def test_sender_skips_catch_up_packet_under_minimum_interval(
    sender: TelemetrySender, monkeypatch: pytest.MonkeyPatch
) -> None:
    _ready_inputs(sender)
    sender._last_send_s = time.monotonic()
    called = []
    monkeypatch.setattr(sender, "_connect", lambda: called.append(True))

    sender._tick()

    assert not called
    assert sender._sent_count == 0
