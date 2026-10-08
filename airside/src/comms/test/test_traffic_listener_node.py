from __future__ import annotations

import json
import threading
from types import SimpleNamespace

import pytest
import rclpy
import websocket
from comms.constants import CONNECTION_TOKEN_ENV, OWN_AIRCRAFT_INDEX_ENV
from comms.traffic_listener_node import TrafficListener, parse_own_aircraft_index


@pytest.fixture
def listener(
    monkeypatch: pytest.MonkeyPatch,
) -> TrafficListener:
    monkeypatch.setenv(CONNECTION_TOKEN_ENV, "test-token")
    monkeypatch.setenv(OWN_AIRCRAFT_INDEX_ENV, "2")
    monkeypatch.setattr(threading.Thread, "start", lambda self: None)
    rclpy.init()
    node = TrafficListener()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize("raw", ["", "-1", "256", "1.5", "goose"])
def test_own_aircraft_index_must_be_explicit_uint8(raw: str) -> None:
    with pytest.raises(ValueError):
        parse_own_aircraft_index(raw)


def test_invalid_websocket_message_invalidates_snapshot(listener: TrafficListener) -> None:
    listener._handle_message("not-json")

    delivery = listener._deliveries.get_nowait()
    assert delivery.connected
    assert not delivery.healthy
    assert delivery.reason == "INVALID_AEAC_MESSAGE"


def test_mock_websocket_snapshot_filters_own_aircraft(
    listener: TrafficListener, monkeypatch: pytest.MonkeyPatch
) -> None:
    traffic = {
        "event": "traffic",
        "payload": {
            "traffic": [
                {
                    "aircraftIndex": index,
                    "name": f"aircraft-{index}",
                    "horizontalKeepAway": 30.0,
                    "verticalKeepAway": 5.0,
                    "position": {
                        "lat": 43.47,
                        "lon": -80.54,
                        "altitude": 15.0,
                        "speed": 2.0,
                        "direction": 90.0,
                    },
                }
                for index in (2, 3)
            ]
        },
    }

    class FakeConnection:
        def settimeout(self, timeout: float) -> None:
            assert timeout > 0.0

        def recv(self) -> str:
            if not listener._deliveries.empty():
                raise websocket.WebSocketConnectionClosedException()
            return json.dumps(traffic)

        def close(self) -> None:
            pass

    monkeypatch.setattr(websocket, "create_connection", lambda *args, **kwargs: FakeConnection())

    with pytest.raises(websocket.WebSocketConnectionClosedException):
        listener._receive_until_disconnected()

    delivery = listener._deliveries.get_nowait()
    assert delivery.healthy
    assert [aircraft.aircraft_index for aircraft in delivery.aircraft] == [3]


def test_bendy_topic_gets_same_complete_snapshot(
    listener: TrafficListener, monkeypatch: pytest.MonkeyPatch
) -> None:
    published = []
    monkeypatch.setattr(listener, "_bendy_pub", SimpleNamespace(publish=published.append))
    listener._handle_message(
        json.dumps(
            {
                "event": "traffic",
                "payload": {
                    "traffic": [
                        {
                            "aircraftIndex": 3,
                            "name": "other",
                            "horizontalKeepAway": 5.0,
                            "verticalKeepAway": 2.0,
                            "position": {
                                "lat": 43.47,
                                "lon": -80.54,
                                "altitude": 15.0,
                                "speed": 1.0,
                                "direction": 90.0,
                            },
                        }
                    ]
                },
            }
        )
    )
    listener._publish_deliveries()

    assert len(published) == 1
    snapshot = published[0]
    assert snapshot.connected and snapshot.healthy
    assert snapshot.own_aircraft_index == 2
    assert len(snapshot.aircraft) == 1
    assert snapshot.aircraft[0].aircraft_index == 3
    assert snapshot.aircraft[0].latitude_deg == 43.47


def test_bendy_topic_empty_and_invalid_snapshots(
    listener: TrafficListener, monkeypatch: pytest.MonkeyPatch
) -> None:
    published = []
    monkeypatch.setattr(listener, "_bendy_pub", SimpleNamespace(publish=published.append))
    listener._handle_message('{"event":"traffic","payload":{"traffic":[]}}')
    listener._handle_message("not-json")
    listener._publish_deliveries()

    assert len(published) == 2
    assert published[0].healthy and not published[0].aircraft
    assert not published[1].healthy and not published[1].aircraft


def test_receive_loop_reconnects_after_socket_loss(
    listener: TrafficListener, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls = 0

    def disconnect() -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            listener._stop.set()
        raise websocket.WebSocketConnectionClosedException()

    monkeypatch.setattr(listener, "_receive_until_disconnected", disconnect)
    monkeypatch.setattr(listener._stop, "wait", lambda timeout: False)

    listener._receive_loop()

    assert calls == 2
    deliveries = []
    while not listener._deliveries.empty():
        deliveries.append(listener._deliveries.get_nowait())
    assert [delivery.reason for delivery in deliveries] == [
        "AEAC_DISCONNECTED",
        "AEAC_DISCONNECTED",
    ]
