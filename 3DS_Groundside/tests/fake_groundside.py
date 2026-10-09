"""A stand-in groundside server for running mock drones by hand.

Logs every message it receives and sends each available drone a random
LOCATION_COMMAND every `--command-interval` seconds.

    python -m tests.fake_groundside --port 8765
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import json
import logging
import math
import random
from dataclasses import dataclass
from typing import Any

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from src.messages import MessageType, MissionState, encode, make_message

logger = logging.getLogger("fake_groundside")


@dataclass
class _DroneView:
    """What groundside knows about one drone connection."""

    mission_state: str | None = None
    task_id: str | None = None
    assigned_task: str | None = None  # last task we sent that wasn't rejected

    @property
    def available(self) -> bool:
        # A drone that just got a command may still report IDLE for its old task,
        # so wait until it reports IDLE for the task we assigned.
        return self.mission_state == MissionState.IDLE and (
            self.assigned_task is None or self.task_id == self.assigned_task
        )


class FakeGroundside:
    """Tracks connected drones and hands random tasks to available ones."""

    def __init__(
        self,
        command_interval: float = 3.0,
        area: float = 50.0,
        altitude: float = 15.0,
        seed: int | None = None,
    ) -> None:
        """Targets are random points in a square of side `area` m at `altitude` m."""
        if not command_interval > 0:  # 0 would busy-loop the command loop
            raise ValueError(
                f"command_interval must be positive, got {command_interval}"
            )
        self._command_interval = command_interval
        self._area = area
        self._altitude = altitude
        self._rng = random.Random(seed)
        self._task_ids = (f"task-{n}" for n in itertools.count(1))
        self.completed_tasks: list[tuple[str, str]] = []  # (drone_id, task_id)
        self.rejected_tasks: list[tuple[str, str]] = []

    async def handle(self, websocket: ServerConnection) -> None:
        """Serve one drone connection: expect REGISTER, then exchange messages."""
        try:
            first = json.loads(await websocket.recv())
            is_register = first["type"] == MessageType.REGISTER
            drone_id = first["drone_id"]
        except (ConnectionClosed, ValueError, KeyError, TypeError):
            is_register, drone_id = False, None
        if not is_register or not isinstance(drone_id, str):
            logger.warning("closing connection that did not start with REGISTER")
            return

        logger.info("%s registered", drone_id)
        # Fresh view per connection: a command sent just before a disconnect may
        # never have arrived.
        view = _DroneView()
        sender = asyncio.ensure_future(self._command_loop(drone_id, view, websocket))
        try:
            async for raw in websocket:
                self._on_message(drone_id, view, json.loads(raw))
        except ConnectionClosed:
            pass
        finally:
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            logger.info("%s disconnected", drone_id)

    def _on_message(
        self, drone_id: str, view: _DroneView, message: dict[str, Any]
    ) -> None:
        payload = message["payload"]
        if message["type"] == MessageType.COMMAND_ACK:
            if payload["accepted"]:
                logger.info("%s accepted %s", drone_id, payload["task_id"])
            else:
                logger.warning("%s REJECTED %s", drone_id, payload["task_id"])
                self.rejected_tasks.append((drone_id, payload["task_id"]))
                if view.assigned_task == payload["task_id"]:
                    view.assigned_task = None
        elif message["type"] == MessageType.DRONE_STATE:
            state = payload["mission_state"]
            p = payload["position"]
            logger.info(
                "%s %-7s %-8s (%6.1f, %6.1f, %6.1f)",
                drone_id,
                state,
                payload["task_id"],
                p["x"],
                p["y"],
                p["z"],
            )
            if state == MissionState.ARRIVED:
                self.completed_tasks.append((drone_id, payload["task_id"]))
            view.mission_state = state
            view.task_id = payload["task_id"]

    async def _command_loop(
        self, drone_id: str, view: _DroneView, websocket: ServerConnection
    ) -> None:
        """Every interval, send the drone a new task if it is available."""
        try:
            while True:
                await asyncio.sleep(self._command_interval)
                if view.available:
                    await self._send_task(drone_id, view, websocket)
        except ConnectionClosed:
            pass  # the drone disconnected; handle() cleans up
        except Exception:
            # Without this the drone would silently stop getting tasks.
            logger.exception("%s command loop crashed; no more tasks", drone_id)
            raise

    async def _send_task(
        self, drone_id: str, view: _DroneView, websocket: ServerConnection
    ) -> None:
        """Send a LOCATION_COMMAND to a random target and mark it assigned."""
        task_id = next(self._task_ids)
        target = {
            "x": round(self._rng.uniform(0.0, self._area), 1),
            "y": round(self._rng.uniform(0.0, self._area), 1),
            "z": self._altitude,
        }
        payload = {
            "task_id": task_id,
            "target": target,
            "orientation": {"yaw": round(self._rng.uniform(-math.pi, math.pi), 2)},
        }
        message = make_message(MessageType.LOCATION_COMMAND, drone_id, payload)
        logger.info("-> %s %s to %s", drone_id, task_id, target)
        view.assigned_task = task_id
        await websocket.send(encode(message))


async def _serve_forever(host: str, port: int, groundside: FakeGroundside) -> None:
    async with serve(groundside.handle, host, port):
        logger.info("fake groundside listening on ws://%s:%d", host, port)
        await asyncio.Future()


def main() -> None:
    parser = argparse.ArgumentParser(description="Fake groundside for mock drones.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--command-interval", type=float, default=3.0)
    parser.add_argument("--area", type=float, default=50.0)
    parser.add_argument("--altitude", type=float, default=15.0)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    try:
        groundside = FakeGroundside(
            args.command_interval, args.area, args.altitude, args.seed
        )
    except ValueError as error:
        parser.error(str(error))

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S"
    )
    logging.getLogger("websockets").setLevel(logging.WARNING)
    try:
        asyncio.run(_serve_forever(args.host, args.port, groundside))
    except KeyboardInterrupt:
        logger.info("stopped")


if __name__ == "__main__":
    main()
