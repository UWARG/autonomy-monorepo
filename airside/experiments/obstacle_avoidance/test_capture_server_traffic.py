"""The real-server capture must never send a WebSocket frame."""

import json

import capture_server_traffic as capture


class FakeConnection:
    def __init__(self) -> None:
        self.frames = [
            json.dumps(
                {
                    "event": "traffic",
                    "payload": {
                        "traffic": [
                            {
                                "aircraftIndex": 2,
                                "name": "private-name",
                                "horizontalKeepAway": 5.0,
                                "verticalKeepAway": 4.0,
                                "position": {
                                    "lat": 43.0,
                                    "lon": -80.0,
                                    "altitude": 10.0,
                                    "speed": 1.2,
                                    "direction": 90.0,
                                },
                            }
                        ]
                    },
                }
            )
        ]

    def settimeout(self, _: float) -> None:
        pass

    def recv(self) -> str:
        if self.frames:
            return self.frames.pop()
        raise capture.websocket.WebSocketTimeoutException()

    def send(self, _: str) -> None:
        raise AssertionError("receive-only capture sent a frame")

    def close(self) -> None:
        pass


def test_capture_is_read_only_and_omits_name_and_token(monkeypatch, tmp_path) -> None:
    connection = FakeConnection()
    monkeypatch.setattr(
        capture.websocket, "create_connection", lambda *_args, **_kwargs: connection
    )
    output = tmp_path / "traffic.jsonl"

    assert capture.capture(output, 0.01, "wss://example.test", "secret-token") == 0
    contents = output.read_text(encoding="utf-8")
    assert "private-name" not in contents
    assert "secret-token" not in contents
    assert json.loads(contents)["aircraft"][0]["speed_mps"] == 1.2
