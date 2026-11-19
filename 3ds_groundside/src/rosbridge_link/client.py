"""WebSocket session for the 3DS mission topics."""

from __future__ import annotations

import asyncio
import json
import math
from dataclasses import dataclass
from typing import Any

from websockets.asyncio.client import ClientConnection
from websockets.exceptions import ConnectionClosed

HEARTBEAT = "/heartbeat"
EMPTY_TYPE = "std_msgs/msg/Empty"
SETPOINT = "/mission/setpoint"
GO_HOME = "/mission/go_home"
COMMAND_TYPES = {
    SETPOINT: "airside_interfaces/msg/Coordinate",
    GO_HOME: EMPTY_TYPE,
}
HEARTBEAT_TIMEOUT = 3.0
SEND_TIMEOUT = 2.0


class LinkError(Exception):
    """The bridge or airside heartbeat isn't available."""


class CommandDeliveryUnknown(LinkError):
    """A publish may have reached the bridge before the connection failed."""


@dataclass(frozen=True)
class Setpoint:
    lat: float
    lon: float
    alt: float

    def __post_init__(self) -> None:
        if not -90 <= self.lat <= 90 or not -180 <= self.lon <= 180:
            raise ValueError("Latitude must be within ±90° and longitude within ±180°.")
        if not math.isfinite(self.alt) or self.alt < 2.0:
            raise ValueError("Relative altitude must be finite and at least 2 m.")


class RosbridgeSession:
    """One connection. Heartbeat state and publisher registrations are connection-local."""

    def __init__(self, websocket: ClientConnection) -> None:
        self._websocket = websocket
        self._last_heartbeat: float | None = None

    async def _send(self, **message: Any) -> None:
        await asyncio.wait_for(
            self._websocket.send(json.dumps(message, allow_nan=False)), SEND_TIMEOUT
        )

    async def subscribe(self) -> None:
        await self._send(op="subscribe", topic=HEARTBEAT, type=EMPTY_TYPE)

    async def wait_for_heartbeat(self) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + HEARTBEAT_TIMEOUT
        try:
            while (remaining := deadline - loop.time()) > 0:
                raw = await asyncio.wait_for(self._websocket.recv(), remaining)
                try:
                    message = json.loads(raw)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(message, dict):
                    continue
                if message.get("op") == "status" and message.get("level") == "error":
                    raise LinkError(
                        f"Rosbridge reported an error: {message.get('msg')}"
                    )
                if (
                    message.get("op") == "publish"
                    and message.get("topic") == HEARTBEAT
                    and message.get("msg") == {}
                ):
                    self._last_heartbeat = loop.time()
                    return
        except asyncio.TimeoutError:
            pass
        raise LinkError("Airside heartbeat timed out.")

    async def send_setpoint(self, setpoint: Setpoint) -> None:
        await self._publish(
            SETPOINT, {"lat": setpoint.lat, "lon": setpoint.lon, "alt": setpoint.alt}
        )

    async def go_home(self) -> None:
        await self._publish(GO_HOME, {})

    async def _publish(self, topic: str, payload: dict[str, float]) -> None:
        if (
            self._last_heartbeat is None
            or asyncio.get_running_loop().time() - self._last_heartbeat
            >= HEARTBEAT_TIMEOUT
        ):
            raise LinkError("A fresh airside heartbeat is required to send a command.")

        try:
            await self._send(op="publish", topic=topic, msg=payload)
        except (ConnectionClosed, OSError, asyncio.TimeoutError) as error:
            raise CommandDeliveryUnknown(
                "Connection failed during publish. Command delivery is unknown."
            ) from error

    async def advertise_setpoint(self) -> None:
        await self._advertise(SETPOINT)

    async def advertise_go_home(self) -> None:
        await self._advertise(GO_HOME)

    async def _advertise(self, topic: str) -> None:
        await self._send(op="advertise", topic=topic, type=COMMAND_TYPES[topic])
