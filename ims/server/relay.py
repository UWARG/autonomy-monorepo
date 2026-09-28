"""
IMS relay: fans airside messages out to dashboards.

Airside senders connect to /airside; browsers connect to /client. Every valid
airside message is rebroadcast unchanged to all clients, and the latest message
per type is replayed to clients that connect later.

Run from the monorepo root:
    python -m ims.server.relay [--host 127.0.0.1] [--port 8765]
"""

import argparse
import asyncio
import logging

import msgspec
from websockets.asyncio.server import broadcast, serve

from utils.src.message_decoder import decode

AIRSIDE_PATH = "/airside"
CLIENT_PATH = "/client"
POLICY_VIOLATION = 1008

log = logging.getLogger("ims.relay")


class Relay:
    def __init__(self) -> None:
        self._clients = set()
        self._airside_count = 0
        self._latest: dict[str, str] = {}

    async def handler(self, websocket) -> None:
        path = websocket.request.path
        if path == CLIENT_PATH:
            await self._serve_client(websocket)
        elif path == AIRSIDE_PATH:
            await self._serve_airside(websocket)
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

    async def _serve_airside(self, websocket) -> None:
        self._airside_count += 1
        try:
            async for raw in websocket:
                try:
                    message = decode(raw)
                except msgspec.DecodeError as error:
                    log.warning("dropping invalid airside message: %s", error)
                    continue
                text = raw if isinstance(raw, str) else raw.decode()
                self._latest[type(message).__struct_config__.tag] = text
                broadcast(self._clients, text)
        finally:
            self._airside_count -= 1
            if self._airside_count == 0:
                self._latest.clear()  # a replayed snapshot would look live


async def main(host: str, port: int) -> None:
    relay = Relay()
    async with serve(relay.handler, host, port):
        log.info("relay listening on ws://%s:%d (%s, %s)", host, port, AIRSIDE_PATH, CLIENT_PATH)
        await asyncio.get_running_loop().create_future()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IMS relay between airside senders and dashboards.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    try:
        asyncio.run(main(args.host, args.port))
    except KeyboardInterrupt:
        pass
