"""
Async AEAC competition WebSocket client, feeding traffic straight into the relay.

No local socket hop: this runs as a background task inside the relay's own
event loop rather than as a separate process. AEAC docs:
https://aeac.mylonics.com/#/connect
"""

import asyncio
import json
import logging
from typing import Callable
from urllib.parse import quote

from websockets.asyncio.client import connect as ws_connect
from websockets.exceptions import WebSocketException

from utils.src.messages import NearbyDronePayload

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


async def run_aeac_feed(
    url: str,
    token: str,
    on_drones: Callable[[list[NearbyDronePayload]], None],
    on_disconnected: Callable[[], None],
) -> None:
    """
    Connects to AEAC and calls on_drones() for every traffic event. Reconnects
    forever on failure; on_disconnected() runs after every disconnect (clean or
    not) so a relay's replayed-to-late-joiners cache doesn't look live.
    """
    separator = "&" if "?" in url else "?"
    full_url = f"{url}{separator}Authorization={quote(token, safe='')}"
    while True:
        try:
            async with ws_connect(full_url) as ws:
                log.info("connected to AEAC")
                async for raw in ws:
                    message = json.loads(raw)
                    event = message.get("event")
                    if event == "traffic":
                        on_drones(traffic_to_drones(message["payload"]))
                    elif event == "infraction":
                        log.info("[infraction] %s", message)
                    elif event == "error":
                        log.warning("[error] %s", message.get("message"))
        except (WebSocketException, OSError) as error:
            log.warning("AEAC connection lost (%s); reconnecting in %ss", error, RECONNECT_DELAY_S)
        on_disconnected()
        await asyncio.sleep(RECONNECT_DELAY_S)
