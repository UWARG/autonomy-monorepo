from __future__ import annotations

import asyncio
import json
import threading
import unittest
from typing import Any

from aeac_bridge.client import AeacClient
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve


class LiveSocketTests(unittest.IsolatedAsyncioTestCase):
    async def test_local_server_carries_traffic_and_telemetry_on_one_socket(
        self,
    ) -> None:
        stop = threading.Event()
        traffic_received = asyncio.Event()
        telemetry_received = asyncio.Event()
        received_telemetry: list[dict[str, Any]] = []

        async def handler(websocket: Any) -> None:
            await websocket.send(
                '{"event":"traffic","payload":{"traffic":[]}}'
            )
            received_telemetry.append(json.loads(await websocket.recv()))
            telemetry_received.set()
            while not stop.is_set():
                await asyncio.sleep(0.01)

        async with serve(handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            client = AeacClient(
                base_url=f"ws://127.0.0.1:{port}/test",
                token="local-token",
                stop_signal=stop,
                telemetry_provider=lambda: {"uavId": "WARG-01"},
                on_connected=lambda: None,
                on_disconnected=lambda _reason: None,
                on_traffic=lambda _traffic: traffic_received.set(),
                on_protocol_error=lambda _reason: None,
                on_server_event=lambda _event, _detail: None,
                connector=connect,
                initial_backoff_s=0.01,
                maximum_backoff_s=0.02,
            )
            client_task = asyncio.create_task(client.run())
            await asyncio.wait_for(traffic_received.wait(), timeout=1.0)
            await asyncio.wait_for(telemetry_received.wait(), timeout=1.0)
            stop.set()
            await asyncio.wait_for(client_task, timeout=1.0)

        self.assertEqual(received_telemetry[0]["action"], "telemetry")
        self.assertEqual(received_telemetry[0]["data"]["uavId"], "WARG-01")


if __name__ == "__main__":
    unittest.main()
