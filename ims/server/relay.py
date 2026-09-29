"""
IMS relay: connects to AEAC directly and fans traffic out to dashboards.

Browsers connect to /client. Every AEAC traffic update is rebroadcast to all
connected clients, and the latest snapshot is replayed to clients that
connect later.

Reads AEAC_CONNECTION_TOKEN from the environment, or from ims/.env (gitignored,
one developer's own token) if set there instead.

Run from the monorepo root:
    python -m ims.server.relay [--host 127.0.0.1] [--port 8765]
"""

import argparse
import asyncio
import logging
import os
from pathlib import Path

from dotenv import load_dotenv
from websockets.asyncio.server import broadcast, serve

from utils.src.message_encoder import encode_nearby_drones

from .aeac import run_aeac_feed

load_dotenv(Path(__file__).resolve().parent.parent / ".env")

DEFAULT_AEAC_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"

CLIENT_PATH = "/client"
POLICY_VIOLATION = 1008

log = logging.getLogger("ims.relay")


class Relay:
    def __init__(self) -> None:
        self._clients = set()
        self._latest: dict[str, str] = {}
        self._aeac_task: asyncio.Task | None = None

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


async def main(host: str, port: int) -> None:
    token = os.environ.get("AEAC_CONNECTION_TOKEN", "").strip()
    if not token:
        raise SystemExit("Set AEAC_CONNECTION_TOKEN to the token from the AEAC connect page.")
    aeac_url = os.environ.get("AEAC_WEBSOCKET_URL", DEFAULT_AEAC_URL)

    relay = Relay()
    # Held on the instance: an unreferenced Task can be garbage-collected mid-run.
    relay._aeac_task = asyncio.create_task(
        run_aeac_feed(aeac_url, token, relay.publish_nearby_drones, relay.clear_nearby_drones)
    )
    async with serve(relay.handler, host, port):
        log.info("relay listening on ws://%s:%d (%s)", host, port, CLIENT_PATH)
        await asyncio.get_running_loop().create_future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IMS relay: AEAC feed fanned out to dashboards.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    try:
        asyncio.run(main(args.host, args.port))
    except KeyboardInterrupt:
        pass
