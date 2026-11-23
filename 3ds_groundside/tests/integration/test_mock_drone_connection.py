"""Runs a MockDrone against an in-process fake groundside server."""

from __future__ import annotations

import asyncio
import json
from collections.abc import Awaitable
from typing import Any, Callable

import pytest
from websockets.asyncio.server import ServerConnection, serve

from src.mock_drone_fleet import mock_drone
from src.mock_drone_fleet.geometry import GeoPosition, from_local
from src.mock_drone_fleet.mock_drone import MockDrone
from tests.fakes import make_location_command

Handler = Callable[[ServerConnection], Awaitable[None]]

HOME = GeoPosition(43.4723, -80.5449, 0.0)
# 5 m away (3 m east, 4 m north) at 15 m up: about 0.5 s of flight at 10 m/s.
TARGET = from_local(HOME, (3.0, 4.0, 15.0))._asdict()


@pytest.fixture(autouse=True)
def _fast_reconnect(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(mock_drone, "RECONNECT_MIN_DELAY_S", 0.05)
    monkeypatch.setattr(mock_drone, "RECONNECT_MAX_DELAY_S", 0.05)


def _fast_drone() -> MockDrone:
    """A drone that reports every 50 ms, to keep tests short."""
    return MockDrone(
        "drone-01",
        start_position=from_local(HOME, (0.0, 0.0, 15.0)),
        speed=10.0,
        report_period=0.05,
    )


async def _run_against(
    groundside: Handler, done: asyncio.Event, received: list[dict[str, Any]]
) -> None:
    """Run a fast drone against `groundside` until it sets `done`."""
    async with serve(groundside, "127.0.0.1", 0) as server:
        port = server.sockets[0].getsockname()[1]
        try:
            await asyncio.wait_for(
                _fast_drone().run(f"ws://127.0.0.1:{port}", done.is_set), 5.0
            )
        except asyncio.TimeoutError:
            types = [(m["type"], m["payload"].get("mission_state")) for m in received]
            pytest.fail(f"groundside never finished; received {types}")


def _payloads(received: list[dict[str, Any]], msg_type: str) -> list[dict[str, Any]]:
    return [m["payload"] for m in received if m["type"] == msg_type]


def test_drone_registers_acks_flies_and_stays_arrived() -> None:
    received: list[dict[str, Any]] = []

    async def scenario() -> None:
        done = asyncio.Event()

        async def groundside(websocket: ServerConnection) -> None:
            received.append(json.loads(await websocket.recv()))  # REGISTER
            target = TARGET
            for task_id in ("task-1", "task-2"):
                command = make_location_command(
                    "drone-01", task_id=task_id, target=target
                )
                await websocket.send(json.dumps(command))

            arrived_reports = 0
            async for raw in websocket:
                message = json.loads(raw)
                received.append(message)
                if message["payload"].get("mission_state") == "ARRIVED":
                    arrived_reports += 1
                    if arrived_reports == 2:
                        done.set()
                        return

        await _run_against(groundside, done, received)

    asyncio.run(scenario())

    assert received[0]["type"] == "REGISTER"
    assert received[0]["drone_id"] == "drone-01"
    assert _payloads(received, "COMMAND_ACK") == [
        {"task_id": "task-1", "accepted": True},
        {"task_id": "task-2", "accepted": False},  # still MOVING on task-1
    ]

    states = _payloads(received, "DRONE_STATE")
    mission_states = [s["mission_state"] for s in states]
    assert "MOVING" in mission_states
    assert mission_states[-2:] == ["ARRIVED", "ARRIVED"]

    for arrived in states[-2:]:
        assert arrived["task_id"] == "task-1"
        assert arrived["position"] == TARGET


def test_drone_reregisters_and_continues_mission_after_disconnect() -> None:
    received: list[dict[str, Any]] = []
    connections = 0

    async def scenario() -> None:
        done = asyncio.Event()

        async def groundside(websocket: ServerConnection) -> None:
            nonlocal connections
            connections += 1
            received.append(json.loads(await websocket.recv()))  # REGISTER

            if connections == 1:
                # Start a 5 m flight (0.5 s), then drop the drone mid-flight.
                target = TARGET
                command = make_location_command(
                    "drone-01", task_id="task-1", target=target
                )
                await websocket.send(json.dumps(command))
                async for raw in websocket:
                    message = json.loads(raw)
                    received.append(message)
                    if message["payload"].get("mission_state") == "MOVING":
                        return  # closes the connection
            else:
                async for raw in websocket:
                    message = json.loads(raw)
                    received.append(message)
                    if message["payload"].get("mission_state") == "ARRIVED":
                        done.set()
                        return

        await _run_against(groundside, done, received)

    asyncio.run(scenario())

    registers = [m for m in received if m["type"] == "REGISTER"]
    assert connections == 2
    assert len(registers) == 2

    # The mission carried on over the second connection without a new command.
    second_session = received[received.index(registers[1]) :]
    states = _payloads(second_session, "DRONE_STATE")
    assert all(s["task_id"] == "task-1" for s in states)
    assert states[-1]["mission_state"] == "ARRIVED"
    assert states[-1]["position"] == TARGET
