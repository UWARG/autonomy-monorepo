"""SetpointClient against a fake drone over real TCP on localhost."""

from __future__ import annotations

import json
import socket
import socketserver
import threading
import time
from collections.abc import Iterator

import pytest

from src.setpoint_tester.cli import run
from src.setpoint_tester.client import LinkError, SetpointClient
from src.setpoint_tester.protocol import Reply, go_home, setpoint

TIMEOUT_S = 0.5


class _Handler(socketserver.StreamRequestHandler):
    server: _FakeDroneServer

    def handle(self) -> None:
        drone = self.server.drone
        drone.connections += 1
        for line in self.rfile:
            drone.received.append(json.loads(line))
            behavior = drone.behaviors.pop(0) if drone.behaviors else "ok"
            if behavior == "ok":
                self.wfile.write(b'{"ok": true, "message": "published"}\n')
            elif behavior == "ok_then_hang_up":
                self.wfile.write(b'{"ok": true, "message": "published"}\n')
                return
            elif behavior == "garbage":
                self.wfile.write(b"garbage\n")
            elif behavior == "hang_up":
                return
            elif behavior == "silent":
                pass


class _FakeDroneServer(socketserver.ThreadingTCPServer):
    daemon_threads = True

    def __init__(self, drone: FakeDrone) -> None:
        self.drone = drone
        super().__init__(("127.0.0.1", 0), _Handler)

    def shutdown_request(self, request: socket.socket) -> None:
        super().shutdown_request(request)
        self.drone.hung_up.set()


class FakeDrone:
    """Records each command; answers according to `behaviors` (default "ok")."""

    def __init__(self) -> None:
        self.received: list[dict] = []
        self.behaviors: list[str] = []
        self.connections = 0
        self.hung_up = threading.Event()
        self._server = _FakeDroneServer(self)
        self.port = self._server.server_address[1]
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()


@pytest.fixture
def drone() -> Iterator[FakeDrone]:
    drone = FakeDrone()
    yield drone
    drone.close()


@pytest.fixture
def client(drone: FakeDrone) -> Iterator[SetpointClient]:
    client = SetpointClient("127.0.0.1", drone.port, timeout=TIMEOUT_S)
    yield client
    client.close()


def _wait_for_peer_close(client: SetpointClient, drone: FakeDrone) -> None:
    """Wait until the client can see the drone's hang-up, so the next send
    deterministically starts on a closed connection."""
    assert drone.hung_up.wait(2.0)
    deadline = time.monotonic() + 2.0
    while not client._peer_has_closed():
        assert time.monotonic() < deadline
        time.sleep(0.01)


def test_send_setpoint(drone: FakeDrone, client: SetpointClient) -> None:
    reply = client.send(setpoint(43.4339, -80.5776, 10.0))

    assert reply == Reply(ok=True, message="published")
    assert drone.received == [
        {"type": "setpoint", "lat": 43.4339, "lon": -80.5776, "alt": 10.0}
    ]


def test_commands_share_one_connection(drone: FakeDrone, client: SetpointClient) -> None:
    client.send(setpoint(43.1, -80.5, 10.0))
    client.send(setpoint(43.2, -80.5, 10.0))
    client.send(go_home())

    assert [command["type"] for command in drone.received] == [
        "setpoint",
        "setpoint",
        "go_home",
    ]
    assert drone.connections == 1


def test_reconnects_after_drone_hangs_up(drone: FakeDrone, client: SetpointClient) -> None:
    drone.behaviors = ["ok_then_hang_up"]
    assert client.send(go_home()).ok
    _wait_for_peer_close(client, drone)

    assert client.send(go_home()).ok
    assert drone.connections == 2
    assert len(drone.received) == 2


def test_no_reply_times_out_then_reconnects(drone: FakeDrone, client: SetpointClient) -> None:
    drone.behaviors = ["silent"]
    with pytest.raises(LinkError, match="no reply within .*MAY"):
        client.send(go_home())
    assert not client.connected

    assert client.send(go_home()).ok
    assert drone.connections == 2


def test_hang_up_before_reply(drone: FakeDrone, client: SetpointClient) -> None:
    drone.behaviors = ["hang_up"]
    with pytest.raises(LinkError, match="closed before the reply"):
        client.send(go_home())
    assert not client.connected


def test_malformed_reply(drone: FakeDrone, client: SetpointClient) -> None:
    drone.behaviors = ["garbage"]
    with pytest.raises(LinkError, match="not valid JSON"):
        client.send(go_home())
    assert not client.connected


def test_unreachable_drone() -> None:
    with socket.socket() as unused:
        unused.bind(("127.0.0.1", 0))
        port = unused.getsockname()[1]
    client = SetpointClient("127.0.0.1", port, timeout=TIMEOUT_S)

    with pytest.raises(LinkError, match="cannot connect"):
        client.send(go_home())
    assert not client.connected


def test_cli_session(drone: FakeDrone, client: SetpointClient) -> None:
    lines = iter(["goto 43.4339, -80.5776, 10", "home", "quit"])
    output: list[str] = []

    run(client, lambda prompt: next(lines), output.append)

    assert drone.received == [setpoint(43.4339, -80.5776, 10.0), go_home()]
    assert output.count("OK: published") == 2
