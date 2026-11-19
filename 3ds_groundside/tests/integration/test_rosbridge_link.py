"""Exercise the groundside CLI against a local rosbridge protocol peer."""

import asyncio
import json

import pytest
from websockets.asyncio.server import serve

from rosbridge_link import __main__ as cli
from rosbridge_link import client
from rosbridge_link.client import (
    CommandDeliveryUnknown,
    LinkError,
    RosbridgeSession,
    Setpoint,
)

HEARTBEAT = {"op": "publish", "topic": "/heartbeat", "msg": {}}


def test_mission_messages() -> None:
    async def exercise() -> None:
        messages = asyncio.Queue()

        async def peer(websocket):
            await messages.put(json.loads(await websocket.recv()))
            await messages.put(json.loads(await websocket.recv()))
            await websocket.send(json.dumps(HEARTBEAT))
            await websocket.send(json.dumps(HEARTBEAT))
            await messages.put(json.loads(await websocket.recv()))

        async with serve(peer, "127.0.0.1", 0) as server:
            url = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}"
            cases = [
                (
                    Setpoint(43.0, -80.0, 10.0),
                    "/mission/setpoint",
                    "airside_interfaces/msg/Coordinate",
                    {"lat": 43.0, "lon": -80.0, "alt": 10.0},
                ),
                (None, "/mission/go_home", "std_msgs/msg/Empty", {}),
            ]
            for setpoint, topic, message_type, payload in cases:
                await cli._command(url, setpoint)
                assert await messages.get() == {
                    "op": "advertise",
                    "topic": topic,
                    "type": message_type,
                }
                assert await messages.get() == {
                    "op": "subscribe",
                    "topic": "/heartbeat",
                    "type": "std_msgs/msg/Empty",
                }
                assert await messages.get() == {
                    "op": "publish",
                    "topic": topic,
                    "msg": payload,
                }

    asyncio.run(asyncio.wait_for(exercise(), 5))


def test_watch_reconnects_after_heartbeat_loss(monkeypatch, capsys) -> None:
    monkeypatch.setattr(client, "HEARTBEAT_TIMEOUT", 0.1)
    monkeypatch.setattr(cli, "RECONNECT_DELAY", 0.01)

    async def exercise() -> None:
        subscriptions = asyncio.Queue()

        async def peer(websocket):
            await subscriptions.put(json.loads(await websocket.recv()))
            await websocket.send(json.dumps(HEARTBEAT))
            await websocket.wait_closed()

        async with serve(peer, "127.0.0.1", 0) as server:
            url = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}"
            watcher = asyncio.create_task(cli._status(url, watch=True))
            try:
                for _ in range(2):
                    assert (await subscriptions.get())["op"] == "subscribe"
                # The second session must also receive a heartbeat before going online.
                await subscriptions.get()
            finally:
                watcher.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await watcher

    asyncio.run(asyncio.wait_for(exercise(), 5))
    output = capsys.readouterr()
    assert output.out.count("Online:") >= 2
    assert "Offline: Airside heartbeat timed out." in output.err


def test_command_requires_heartbeat_and_is_never_retried(monkeypatch) -> None:
    monkeypatch.setattr(client, "HEARTBEAT_TIMEOUT", 0.1)

    class FailingSocket:
        def __init__(self) -> None:
            self.attempts = 0

        async def send(self, _message):
            self.attempts += 1
            raise OSError("link lost")

        async def recv(self):
            return json.dumps(HEARTBEAT)

    async def exercise() -> None:
        socket = FailingSocket()
        session = RosbridgeSession(socket)
        with pytest.raises(LinkError, match="fresh airside heartbeat"):
            await session.go_home()
        assert socket.attempts == 0

        await session.wait_for_heartbeat()
        await asyncio.sleep(0.11)
        with pytest.raises(LinkError, match="fresh airside heartbeat"):
            await session.go_home()
        assert socket.attempts == 0

        await session.wait_for_heartbeat()
        with pytest.raises(CommandDeliveryUnknown):
            await session.go_home()
        assert socket.attempts == 1

    asyncio.run(exercise())


@pytest.mark.parametrize(
    "coordinates", [(91, 0, 5), (0, -181, 5), (0, 0, 1), (0, 0, float("inf"))]
)
def test_invalid_setpoints(coordinates) -> None:
    with pytest.raises(ValueError):
        Setpoint(*coordinates)
