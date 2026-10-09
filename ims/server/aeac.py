"""
Async AEAC competition WebSocket client, run inside the relay's event loop.
"""

import asyncio
import json
import logging
import time
from typing import Callable
from urllib.parse import quote

from websockets.asyncio.client import ClientConnection, connect as ws_connect
from websockets.exceptions import WebSocketException

from utils.src.messages import AeacAckPayload, AeacInfractionPayload, NearbyDronePayload

log = logging.getLogger("ims.aeac")

RECONNECT_DELAY_S = 5


def traffic_to_drones(payload: dict) -> list[NearbyDronePayload]:
    """Convert an AEAC `traffic` event payload into IMS drone payloads."""
    drones = []
    for aircraft in payload["traffic"]:
        position = aircraft["position"]
        drones.append(
            NearbyDronePayload(
                id=int(aircraft["aircraftIndex"]),
                name=aircraft["name"],
                lat=float(position["lat"]),
                lon=float(position["lon"]),
                alt=float(position["altitude"]),
                speed=float(position["speed"]),
                direction=float(position["direction"]),
                horizontal_keep_away=float(aircraft["horizontalKeepAway"]),
                vertical_keep_away=float(aircraft["verticalKeepAway"]),
            )
        )
    return drones


def ack_to_payload(message: dict) -> AeacAckPayload:
    """Convert an AEAC `telemetry_ack` event (our packet echoed back plus AEAC's checks)."""
    return AeacAckPayload(
        unix_time=float(message["unixTime"]),
        inside_boundary=bool(message["insideBoundary"]),
        too_close_to_traffic=bool(message["tooCloseToTraffic"]),
    )


# Fields of an `infraction` event that aren't per-type counters.
_INFRACTION_NON_COUNTERS = {"event", "uavId", "siteId", "last_infraction", "score", "armed_seconds"}


def infraction_to_payload(message: dict) -> AeacInfractionPayload:
    """Convert an AEAC `infraction` event (cumulative counters for this UAV)."""
    return AeacInfractionPayload(
        last_infraction=str(message["last_infraction"]),
        counts={
            key: value
            for key, value in message.items()
            if key not in _INFRACTION_NON_COUNTERS and isinstance(value, int)
        },
        armed_seconds=float(message["armed_seconds"]),
        received_at=time.time(),
    )


async def run_aeac_feed(
    url: str,
    token: str,
    on_drones: Callable[[list[NearbyDronePayload]], None],
    on_ack: Callable[[AeacAckPayload], None],
    on_infraction: Callable[[AeacInfractionPayload], None],
    on_connected: Callable[[ClientConnection], None],
    on_disconnected: Callable[[], None],
) -> None:
    """Connects to AEAC, dispatches each event to its callback, and reconnects forever."""
    separator = "&" if "?" in url else "?"
    full_url = f"{url}{separator}Authorization={quote(token, safe='')}"
    while True:
        try:
            async with ws_connect(full_url) as ws:
                log.info("connected to AEAC")
                on_connected(ws)
                async for raw in ws:
                    message = json.loads(raw)
                    event = message.get("event")
                    if event == "traffic":
                        on_drones(traffic_to_drones(message["payload"]))
                    elif event == "telemetry_ack":
                        on_ack(ack_to_payload(message))
                    elif event == "infraction":
                        log.warning("[infraction] %s", message)
                        on_infraction(infraction_to_payload(message))
                    elif event == "error":
                        log.warning("[error] %s", message.get("message"))
        except (WebSocketException, OSError) as error:
            log.warning("AEAC connection lost (%s); reconnecting in %ss", error, RECONNECT_DELAY_S)
        on_disconnected()
        await asyncio.sleep(RECONNECT_DELAY_S)
