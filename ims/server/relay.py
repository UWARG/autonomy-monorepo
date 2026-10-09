"""
IMS relay: fans AEAC traffic out to dashboards and sends our telemetry to AEAC at 1 Hz.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import time
from pathlib import Path

from dotenv import load_dotenv
from websockets.asyncio.client import ClientConnection
from websockets.asyncio.server import broadcast, serve
from websockets.exceptions import WebSocketException

from utils.src.message_encoder import (
    encode_aeac_ack,
    encode_aeac_infraction,
    encode_nearby_drones,
    encode_telemetry_sent,
)
from utils.src.messages import AeacAckPayload, AeacInfractionPayload

from .aeac import run_aeac_feed
from .drone import DroneState, run_drone_feed
from .telemetry import LINK_TIMEOUT_S, build_packet

load_dotenv(Path(__file__).resolve().parent.parent / ".env")

DEFAULT_AEAC_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"
DEFAULT_ROSBRIDGE_URL = "ws://127.0.0.1:9090"
DEFAULT_UAV_ID = "WARG-01"

TELEMETRY_PERIOD_S = 1.0  # AEAC penalizes packet gaps > 1.1s and < 0.4s

CLIENT_PATH = "/client"
POLICY_VIOLATION = 1008

log = logging.getLogger("ims.relay")


class Relay:
    def __init__(self) -> None:
        self._clients = set()
        self._latest: dict[str, str] = {}
        self._aeac_ws: ClientConnection | None = None
        self.drone = DroneState()
        # Held on the instance: an unreferenced Task can be garbage-collected mid-run.
        self._tasks: list[asyncio.Task] = []

    async def handler(self, websocket) -> None:
        if websocket.request.path == CLIENT_PATH:
            await self._serve_client(websocket)
        else:
            await websocket.close(POLICY_VIOLATION, "unknown path")

    async def _serve_client(self, websocket) -> None:
        self._clients.add(websocket)
        try:
            for text in list(self._latest.values()):
                await websocket.send(text)
            await websocket.wait_closed()
        finally:
            self._clients.discard(websocket)

    def publish_nearby_drones(self, drones) -> None:
        text = encode_nearby_drones(drones).decode()
        self._latest["nearby_drones"] = text
        broadcast(self._clients, text)

    def clear_nearby_drones(self) -> None:
        self._latest.pop("nearby_drones", None)  # a replayed snapshot would look live

    def publish_aeac_ack(self, ack: AeacAckPayload) -> None:
        # Not cached for replay: a late joiner would see an old ack as fresh.
        broadcast(self._clients, encode_aeac_ack(ack).decode())

    def publish_aeac_infraction(self, infraction: AeacInfractionPayload) -> None:
        text = encode_aeac_infraction(infraction).decode()
        self._latest["aeac_infraction"] = text  # cumulative counts, still true for late joiners
        broadcast(self._clients, text)

    def on_aeac_connected(self, ws: ClientConnection) -> None:
        self._aeac_ws = ws

    def on_aeac_disconnected(self) -> None:
        self._aeac_ws = None
        self.clear_nearby_drones()

    async def send_telemetry(self, uav_id: str) -> None:
        """Sends one packet per TELEMETRY_PERIOD_S on a fixed schedule, link lost or not."""
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        while True:
            # Never catch up on missed ticks: a burst would trip AEAC's <0.4s penalty.
            next_tick = max(next_tick + TELEMETRY_PERIOD_S, loop.time())
            await asyncio.sleep(next_tick - loop.time())

            age = self.drone.age_s()
            packet = build_packet(
                uav_id,
                self.drone.messages,
                link_ok=age is not None and age <= LINK_TIMEOUT_S,
                now=time.time(),
            )
            ws = self._aeac_ws
            if ws is None or packet is None:
                continue
            try:
                await ws.send(json.dumps({"action": "telemetry", "data": packet}))
            except WebSocketException as error:
                log.warning("telemetry send failed (%s)", error)
                continue
            # Not cached for replay: a late joiner would see an old packet as fresh.
            broadcast(self._clients, encode_telemetry_sent(packet).decode())


async def main(host: str, port: int) -> None:
    token = os.environ.get("AEAC_CONNECTION_TOKEN", "").strip()
    if not token:
        raise SystemExit("Set AEAC_CONNECTION_TOKEN to the token from the AEAC connect page.")
    aeac_url = os.environ.get("AEAC_WEBSOCKET_URL", DEFAULT_AEAC_URL)
    rosbridge_url = os.environ.get("ROSBRIDGE_URL", DEFAULT_ROSBRIDGE_URL)
    uav_id = os.environ.get("AEAC_UAV_ID", DEFAULT_UAV_ID)

    relay = Relay()
    relay._tasks = [
        asyncio.create_task(
            run_aeac_feed(
                aeac_url,
                token,
                relay.publish_nearby_drones,
                relay.publish_aeac_ack,
                relay.publish_aeac_infraction,
                relay.on_aeac_connected,
                relay.on_aeac_disconnected,
            )
        ),
        asyncio.create_task(run_drone_feed(rosbridge_url, relay.drone)),
        asyncio.create_task(relay.send_telemetry(uav_id)),
    ]
    async with serve(relay.handler, host, port):
        log.info("relay listening on ws://%s:%d (%s)", host, port, CLIENT_PATH)
        await asyncio.get_running_loop().create_future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IMS relay: AEAC traffic in, our telemetry out.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    try:
        asyncio.run(main(args.host, args.port))
    except KeyboardInterrupt:
        pass
