"""Smoke test: a mock drone completes tasks handed out by the fake groundside."""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
from typing import Any

import pytest
from websockets.asyncio.client import ClientConnection, connect
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from src.messages import MissionState, drone_state, encode, register
from src.mock_drone import MockDrone
from tests.fake_groundside import FakeGroundside


def test_fake_groundside_drives_drone_through_several_tasks() -> None:
    groundside = FakeGroundside(command_interval=0.1, area=5.0, seed=1)

    async def scenario() -> None:
        async with serve(groundside.handle, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            drone = MockDrone(
                "drone-01", (0.0, 0.0, 15.0), speed=50.0, report_period=0.02
            )
            try:
                await asyncio.wait_for(
                    drone.run(
                        f"ws://127.0.0.1:{port}",
                        lambda: len(groundside.completed_tasks) >= 3,
                    ),
                    5.0,
                )
            except asyncio.TimeoutError:
                pytest.fail(
                    f"timed out; completed={groundside.completed_tasks} "
                    f"rejected={groundside.rejected_tasks}"
                )

    asyncio.run(scenario())

    assert groundside.completed_tasks == [
        ("drone-01", "task-1"),
        ("drone-01", "task-2"),
        ("drone-01", "task-3"),
    ]
    # Groundside must never double-book a drone that is still busy.
    assert groundside.rejected_tasks == []


def test_duplicate_registration_closes_old_connection_and_serves_new_one(
    caplog: pytest.LogCaptureFixture,
) -> None:
    groundside = FakeGroundside(command_interval=0.05)
    idle = drone_state("drone-01", None, MissionState.IDLE, (0, 0, 15), (0, 0, 0))

    async def wait_until_registered() -> None:
        while "drone-01" not in groundside._connections:
            await asyncio.sleep(0.01)

    async def drain_until_closed(websocket: ClientConnection) -> None:
        with contextlib.suppress(ConnectionClosed):
            async for _ in websocket:
                pass  # may get a command before being replaced

    async def scenario() -> dict[str, Any]:
        async with serve(groundside.handle, "127.0.0.1", 0) as server:
            url = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}"
            async with connect(url) as old, connect(url) as new:
                await old.send(encode(register("drone-01")))
                await wait_until_registered()
                await new.send(encode(register("drone-01")))
                await new.send(encode(idle))

                await drain_until_closed(old)
                return json.loads(await new.recv())

    with caplog.at_level(logging.INFO, logger="fake_groundside"):
        first_message = asyncio.run(asyncio.wait_for(scenario(), 5.0))

    assert first_message["type"] == "LOCATION_COMMAND"
    assert "drone-01 registered again; closing its old connection" in caplog.text
    assert "drone-01 old connection closed" in caplog.text
