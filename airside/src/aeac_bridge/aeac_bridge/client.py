"""Async AEAC WebSocket worker with bounded reconnect backoff."""

from __future__ import annotations

import asyncio
import inspect
import json
from collections.abc import Callable
from typing import Any, Protocol
from urllib.parse import quote

from aeac_bridge.protocol import TrafficEvent, TrafficProtocolError, parse_traffic_event


class StopSignal(Protocol):
    def is_set(self) -> bool: ...


class AeacClient:
    """Own one bidirectional socket without blocking ROS callbacks."""

    def __init__(
        self,
        *,
        base_url: str,
        token: str,
        stop_signal: StopSignal,
        telemetry_provider: Callable[[], dict[str, Any] | None],
        on_connected: Callable[[], None],
        on_disconnected: Callable[[str], None],
        on_traffic: Callable[[TrafficEvent], None],
        on_protocol_error: Callable[[str], None],
        on_server_event: Callable[[str, str], None],
        connector: Callable[[str], Any] | None = None,
        send_period_s: float = 1.0,
        initial_backoff_s: float = 0.5,
        maximum_backoff_s: float = 5.0,
    ) -> None:
        if send_period_s <= 0.0:
            raise ValueError("send_period_s must be positive")
        if initial_backoff_s <= 0.0 or maximum_backoff_s < initial_backoff_s:
            raise ValueError("invalid reconnect backoff")
        self._base_url = base_url
        self._token = token
        self._stop_signal = stop_signal
        self._telemetry_provider = telemetry_provider
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._on_traffic = on_traffic
        self._on_protocol_error = on_protocol_error
        self._on_server_event = on_server_event
        self._connector = connector or self._default_connector
        self._send_period_s = send_period_s
        self._initial_backoff_s = initial_backoff_s
        self._maximum_backoff_s = maximum_backoff_s

    @property
    def authenticated_url(self) -> str:
        separator = "&" if "?" in self._base_url else "?"
        return f"{self._base_url}{separator}Authorization={quote(self._token, safe='')}"

    @staticmethod
    def _default_connector(url: str) -> Any:
        from websockets.asyncio.client import connect

        return connect(url, open_timeout=10)

    async def run(self) -> None:
        backoff_s = self._initial_backoff_s
        while not self._stop_signal.is_set():
            disconnect_reason = "AEAC_CONNECTION_CLOSED"
            try:
                async with self._connector(self.authenticated_url) as websocket:
                    self._on_connected()
                    backoff_s = self._initial_backoff_s
                    await self._serve(websocket)
            except (OSError, ConnectionError, TimeoutError) as error:
                disconnect_reason = type(error).__name__.upper()
            # This is the outer connection boundary. Third-party WebSocket
            # implementations don't share one stable exception hierarchy.
            except Exception as error:  # noqa: BLE001
                disconnect_reason = type(error).__name__.upper()
            self._on_disconnected(disconnect_reason)
            if self._stop_signal.is_set():
                break
            await self._interruptible_sleep(backoff_s)
            backoff_s = min(self._maximum_backoff_s, backoff_s * 2.0)

    async def _serve(self, websocket: Any) -> None:
        receiver = asyncio.create_task(self._receive_loop(websocket))
        loop = asyncio.get_running_loop()
        next_send_s = loop.time()
        try:
            while not self._stop_signal.is_set():
                if receiver.done():
                    await receiver
                    raise ConnectionError("AEAC receive stream ended")
                now_s = loop.time()
                telemetry = self._telemetry_provider()
                if telemetry is not None and now_s >= next_send_s:
                    await websocket.send(
                        json.dumps({"action": "telemetry", "data": telemetry})
                    )
                    next_send_s = now_s + self._send_period_s
                await asyncio.sleep(0.05)
        finally:
            receiver.cancel()
            try:
                await receiver
            except asyncio.CancelledError:
                pass

    async def _receive_loop(self, websocket: Any) -> None:
        async for raw in websocket:
            try:
                traffic = parse_traffic_event(raw)
            except TrafficProtocolError as error:
                self._on_protocol_error(str(error))
                continue
            if traffic is not None:
                self._on_traffic(traffic)
                continue

            try:
                message = json.loads(raw)
            except (TypeError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            event = message.get("event", "")
            if event in {"error", "infraction"}:
                detail = message.get("message", "")
                self._on_server_event(str(event), str(detail))

    async def _interruptible_sleep(self, duration_s: float) -> None:
        deadline_s = asyncio.get_running_loop().time() + duration_s
        while not self._stop_signal.is_set():
            remaining_s = deadline_s - asyncio.get_running_loop().time()
            if remaining_s <= 0.0:
                return
            await asyncio.sleep(min(0.1, remaining_s))


def connector_is_async_context_manager(
    connector: Callable[[str], Any], url: str
) -> bool:
    """Small validation helper used by dependency-light tests."""

    candidate = connector(url)
    return hasattr(candidate, "__aenter__") and not inspect.isawaitable(candidate)
