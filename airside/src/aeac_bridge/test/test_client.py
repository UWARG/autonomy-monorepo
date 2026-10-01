from __future__ import annotations

import asyncio
import json
import unittest
from collections.abc import Callable
from typing import Any

from aeac_bridge.client import AeacClient

TRAFFIC_MESSAGE = '{"event":"traffic","payload":{"traffic":[]}}'


class _StopSignal:
    def __init__(self) -> None:
        self.stopped = False

    def is_set(self) -> bool:
        return self.stopped


class _Socket:
    def __init__(self, messages: list[str]) -> None:
        self.messages = list(messages)
        self.sent: list[str] = []

    async def send(self, message: str) -> None:
        self.sent.append(message)

    def __aiter__(self):
        return self

    async def __anext__(self) -> str:
        if self.messages:
            return self.messages.pop(0)
        await asyncio.sleep(60.0)
        raise StopAsyncIteration


class _Connection:
    def __init__(self, socket: _Socket, error: Exception | None = None) -> None:
        self.socket = socket
        self.error = error

    async def __aenter__(self) -> _Socket:
        if self.error is not None:
            raise self.error
        return self.socket

    async def __aexit__(self, *_args: object) -> None:
        return None


class AeacClientTests(unittest.IsolatedAsyncioTestCase):
    def make_client(
        self,
        *,
        stop: _StopSignal,
        connector: Callable[[str], Any],
        on_traffic: Callable[[Any], None],
        disconnected: list[str] | None = None,
    ) -> AeacClient:
        disconnected = disconnected if disconnected is not None else []
        return AeacClient(
            base_url="wss://example.invalid/prod?stage=test",
            token="token with spaces&symbols",
            stop_signal=stop,
            telemetry_provider=lambda: {"uavId": "WARG-01"},
            on_connected=lambda: None,
            on_disconnected=disconnected.append,
            on_traffic=on_traffic,
            on_protocol_error=lambda _reason: None,
            on_server_event=lambda _event, _detail: None,
            connector=connector,
            send_period_s=0.01,
            initial_backoff_s=0.001,
            maximum_backoff_s=0.002,
        )

    async def test_one_socket_receives_traffic_and_sends_telemetry(self) -> None:
        stop = _StopSignal()
        socket = _Socket([TRAFFIC_MESSAGE])
        received = []

        def on_traffic(event: Any) -> None:
            received.append(event)
            stop.stopped = True

        client = self.make_client(
            stop=stop,
            connector=lambda _url: _Connection(socket),
            on_traffic=on_traffic,
        )
        await asyncio.wait_for(client.run(), timeout=1.0)

        self.assertEqual(len(received), 1)
        self.assertGreaterEqual(len(socket.sent), 1)
        self.assertEqual(json.loads(socket.sent[0])["action"], "telemetry")
        self.assertIn(
            "Authorization=token%20with%20spaces%26symbols", client.authenticated_url
        )

    async def test_connection_failure_reconnects_with_bounded_backoff(self) -> None:
        stop = _StopSignal()
        socket = _Socket([TRAFFIC_MESSAGE])
        attempts = 0
        disconnected: list[str] = []

        def connector(_url: str) -> _Connection:
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                return _Connection(socket, OSError("offline"))
            return _Connection(socket)

        def on_traffic(_event: Any) -> None:
            stop.stopped = True

        client = self.make_client(
            stop=stop,
            connector=connector,
            on_traffic=on_traffic,
            disconnected=disconnected,
        )
        await asyncio.wait_for(client.run(), timeout=1.0)

        self.assertEqual(attempts, 2)
        self.assertIn("OSERROR", disconnected)

    async def test_protocol_error_does_not_silently_become_clear(self) -> None:
        stop = _StopSignal()
        socket = _Socket(["not-json", TRAFFIC_MESSAGE])
        errors: list[str] = []

        client = AeacClient(
            base_url="wss://example.invalid",
            token="token",
            stop_signal=stop,
            telemetry_provider=lambda: None,
            on_connected=lambda: None,
            on_disconnected=lambda _reason: None,
            on_traffic=lambda _event: setattr(stop, "stopped", True),
            on_protocol_error=errors.append,
            on_server_event=lambda _event, _detail: None,
            connector=lambda _url: _Connection(socket),
            initial_backoff_s=0.001,
            maximum_backoff_s=0.002,
        )
        await asyncio.wait_for(client.run(), timeout=1.0)

        self.assertEqual(errors, ["message must be valid JSON"])

    async def test_stop_signal_shuts_down_idle_socket_worker(self) -> None:
        stop = _StopSignal()
        socket = _Socket([])
        client = self.make_client(
            stop=stop,
            connector=lambda _url: _Connection(socket),
            on_traffic=lambda _event: None,
        )

        task = asyncio.create_task(client.run())
        await asyncio.sleep(0.1)
        stop.stopped = True
        await asyncio.wait_for(task, timeout=1.0)

        self.assertTrue(task.done())


if __name__ == "__main__":
    unittest.main()
