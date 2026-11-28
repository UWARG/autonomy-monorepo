"""
Runs real MockDrones over websockets against a small test groundside, so the
survey executive can be tested end to end in one process.

The drone state manager (#153) is still a skeleton, so
``WebsocketDroneManager`` stands in for it with the two methods the executive
uses. The server and every drone share one background event loop; the test
thread (or a ``SurveyRunner``) calls the manager from outside it.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import logging
import math
import threading
import time
from collections.abc import Callable, Coroutine, Iterable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace
from typing import Any, TypeVar

import numpy as np
from utils.src.types import Attitude, Coordinate
from websockets.asyncio.server import Server, ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from src.groundside_manager.groundside_manager import DroneState, MissionState
from src.image_stitcher.message import ImageMessage
from src.mock_drone_fleet.geometry import GeoPosition, from_local
from src.mock_drone_fleet.messages import MessageType, encode, make_message
from src.mock_drone_fleet.mock_drone import MockDrone
from src.mock_drone_fleet.mock_fleet import (
    DEFAULT_HOME_LATITUDE,
    DEFAULT_HOME_LONGITUDE,
)

logger = logging.getLogger(__name__)

T = TypeVar("T")

DEFAULT_HOME = GeoPosition(DEFAULT_HOME_LATITUDE, DEFAULT_HOME_LONGITUDE, 0.0)
# Fast drones and frequent reports keep a whole survey to a few seconds.
DRONE_SPEED_M_S = 100.0
REPORT_PERIOD_S = 0.02
DRONE_SPACING_M = 5.0
SEND_TIMEOUT_S = 2.0
# Long enough for a drone to notice its stop flag while waiting to reconnect.
STOP_TIMEOUT_S = 5.0
# Nothing in CI reads the pixels, so the synthetic photos are tiny.
PHOTO_SHAPE = (4, 4, 3)


def photo_from_state(state: DroneState) -> ImageMessage:
    """
    The photo a drone sends when it arrives (#233): its reported position and
    attitude, converted from the protocol's radians to the stitcher's degrees,
    named after the task.
    """

    if state.position is None or state.orientation is None:
        raise ValueError(f"{state.drone_id} has not reported a position yet")
    if state.active_task_id is None:
        raise ValueError(f"{state.drone_id} has no task to name the photo after")
    return ImageMessage(
        lat=state.position.lat,
        lon=state.position.lon,
        alt=state.position.alt,
        img=np.zeros(PHOTO_SHAPE, dtype=np.uint8),
        roll_deg=math.degrees(state.orientation.roll),
        pitch_deg=math.degrees(state.orientation.pitch),
        yaw_deg=math.degrees(state.orientation.yaw),
        drone_id=state.drone_id,
        image_id=state.active_task_id,
    )


class WebsocketDroneManager:
    """
    Tracks drones that connect with the mock drone protocol and sends them
    location commands, with the same contract as the planned #153 manager.

    ``on_arrival`` runs on the event loop with a copy of the drone's state
    the first time the drone reports ARRIVED on a task.
    """

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        on_arrival: Callable[[DroneState], None] | None = None,
    ) -> None:
        self._loop = loop
        self._on_arrival = on_arrival
        # Guards everything below: the loop writes, other threads read.
        self._lock = threading.Lock()
        self._states: dict[str, DroneState] = {}
        self._connections: dict[str, ServerConnection] = {}
        self._photographed: dict[str, str] = {}  # drone_id -> last task arrived on
        self._acks: list[tuple[str, str, bool]] = []

    @property
    def acks(self) -> list[tuple[str, str, bool]]:
        """
        Every COMMAND_ACK so far: (drone_id, task_id, accepted).
        """

        with self._lock:
            return list(self._acks)

    def get_all_drones(self) -> dict[str, DroneState]:
        with self._lock:
            return {drone_id: replace(s) for drone_id, s in self._states.items()}

    def send_location_command_sync(
        self, drone_id: str, task_id: str, target: Coordinate, yaw: float
    ) -> bool:
        """
        True if the command was sent, False if the drone is not connected.
        Whether the drone took it shows in its acks and state. Must not be
        called from the event loop, which it waits on.
        """

        if _running_loop() is self._loop:
            raise RuntimeError("send_location_command_sync would block its own loop")
        with self._lock:
            websocket = self._connections.get(drone_id)
        if websocket is None:
            return False
        payload = {
            "task_id": task_id,
            "target": {
                "latitude": target.lat,
                "longitude": target.lon,
                "altitude": target.alt,
            },
            "orientation": {"yaw": yaw},
        }
        message = encode(make_message(MessageType.LOCATION_COMMAND, drone_id, payload))
        future = asyncio.run_coroutine_threadsafe(websocket.send(message), self._loop)
        try:
            future.result(timeout=SEND_TIMEOUT_S)
        except (ConnectionClosed, concurrent.futures.TimeoutError):
            future.cancel()
            return False
        return True

    async def handle(self, websocket: ServerConnection) -> None:
        """
        Serves one drone connection: REGISTER first, then state and acks.
        """

        try:
            first = json.loads(await websocket.recv())
            if first["type"] != MessageType.REGISTER:
                raise ValueError(f"expected REGISTER, got {first['type']!r}")
            drone_id = first["drone_id"]
            vehicle_type = first["payload"]["vehicle_type"]
        except ConnectionClosed:
            return
        except (ValueError, KeyError, TypeError) as error:
            logger.warning("closing a connection that did not register: %r", error)
            await websocket.close()
            return

        self._register(drone_id, vehicle_type, websocket)
        try:
            async for raw in websocket:
                self._handle_message(drone_id, json.loads(raw))
        except ConnectionClosed:
            pass  # an abrupt drop ends the session like a clean close
        finally:
            with self._lock:
                # A reconnect may already have replaced this connection.
                if self._connections.get(drone_id) is websocket:
                    del self._connections[drone_id]
                    self._states[drone_id].connected = False

    def _register(
        self, drone_id: str, vehicle_type: str, websocket: ServerConnection
    ) -> None:
        with self._lock:
            self._connections[drone_id] = websocket
            state = self._states.get(drone_id)
            if state is None:
                self._states[drone_id] = DroneState(
                    drone_id=drone_id,
                    vehicle_type=vehicle_type,
                    position=None,
                    orientation=None,
                    mission_state=MissionState.IDLE,
                    active_task_id=None,
                    last_update_timestamp=None,
                    connected=True,
                )
            else:  # a reconnect keeps the last known state until the next report
                state.connected = True

    def _handle_message(self, drone_id: str, message: dict[str, Any]) -> None:
        payload = message["payload"]
        if message["type"] == MessageType.DRONE_STATE:
            arrival = self._update_state(drone_id, payload)
            if arrival is not None and self._on_arrival is not None:
                self._on_arrival(arrival)
        elif message["type"] == MessageType.COMMAND_ACK:
            with self._lock:
                self._acks.append((drone_id, payload["task_id"], payload["accepted"]))

    def _update_state(
        self, drone_id: str, payload: dict[str, Any]
    ) -> DroneState | None:
        """
        Stores a DRONE_STATE. Returns a copy of the state if it is the
        drone's first ARRIVED on this task.
        """

        position = payload["position"]
        orientation = payload["orientation"]
        with self._lock:
            state = self._states[drone_id]
            state.position = Coordinate(
                position["latitude"], position["longitude"], position["altitude"]
            )
            state.orientation = Attitude(
                orientation["roll"], orientation["pitch"], orientation["yaw"],
                0.0, 0.0, 0.0,
            )  # fmt: skip
            state.mission_state = MissionState(payload["mission_state"])
            state.active_task_id = payload["task_id"]
            # The executive's clock is monotonic too.
            state.last_update_timestamp = time.monotonic()
            task_id = state.active_task_id
            arrived = state.mission_state is MissionState.ARRIVED
            if not arrived or task_id is None:
                return None
            if self._photographed.get(drone_id) == task_id:
                return None
            self._photographed[drone_id] = task_id
            return replace(state)


@dataclass
class _Flyer:
    stop: threading.Event
    run: concurrent.futures.Future[None]


class FleetHarness:
    """
    A ``WebsocketDroneManager`` and MockDrones flying against it, between
    ``start`` and ``close`` (or use ``flying``). Drones start a few metres
    apart, east of ``DEFAULT_HOME``.

    Every arrival becomes a synthetic photo, kept in ``photos``. It then goes
    to ``photo_sink`` if ``keep_photo`` says so, and to ``held`` otherwise.
    Both callbacks run on the event loop.
    """

    def __init__(self, drone_ids: Iterable[str]) -> None:
        self._drone_ids = list(drone_ids)
        self.photo_sink: Callable[[ImageMessage], None] = lambda photo: None
        self.keep_photo: Callable[[ImageMessage], bool] = lambda photo: True
        self._photos_lock = threading.Lock()
        self._photos: list[ImageMessage] = []
        self._held: list[ImageMessage] = []

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, name="fleet-harness", daemon=True
        )
        self.manager = WebsocketDroneManager(self._loop, on_arrival=self._photograph)
        self._server: Server | None = None
        self._flyers: dict[str, _Flyer] = {}

    @property
    def photos(self) -> list[ImageMessage]:
        with self._photos_lock:
            return list(self._photos)

    @property
    def held(self) -> list[ImageMessage]:
        with self._photos_lock:
            return list(self._held)

    def start(self) -> None:
        """
        Starts the server and launches every drone.
        """

        self._thread.start()
        try:
            self._server = self._call(self._serve())
            port = self._server.sockets[0].getsockname()[1]
            for index, drone_id in enumerate(self._drone_ids):
                self._launch(drone_id, index, f"ws://127.0.0.1:{port}")
        except BaseException:
            self.close()
            raise

    def kill(self, drone_id: str) -> None:
        """
        Stops one drone and waits until it has closed its connection.
        """

        flyer = self._flyers[drone_id]
        flyer.stop.set()
        flyer.run.result(timeout=STOP_TIMEOUT_S)

    def wait_for(
        self, condition: Callable[[], bool], timeout_s: float = 10.0, what: str = ""
    ) -> None:
        """
        Polls ``condition`` until it is true, or fails the test.
        """

        deadline = time.monotonic() + timeout_s
        while not condition():
            if time.monotonic() > deadline:
                raise AssertionError(f"timed out after {timeout_s}s waiting for {what}")
            time.sleep(0.01)

    def wait_until_reporting(self, timeout_s: float = 10.0) -> None:
        """
        Waits until every drone is connected and has reported a position.
        """

        def all_reporting() -> bool:
            states = self.manager.get_all_drones()
            return all(
                (state := states.get(drone_id)) is not None
                and state.connected
                and state.position is not None
                for drone_id in self._drone_ids
            )

        self.wait_for(all_reporting, timeout_s, "every drone to report")

    def close(self) -> None:
        """
        Stops every drone, then the server, then the loop. Safe to call twice.
        """

        if not self._thread.is_alive():
            return
        try:
            for flyer in self._flyers.values():
                flyer.stop.set()
            for flyer in self._flyers.values():
                flyer.run.result(timeout=STOP_TIMEOUT_S)
            if self._server is not None:
                self._server.close()
                self._call(self._server.wait_closed())
        finally:
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(STOP_TIMEOUT_S)
            if not self._thread.is_alive():  # closing a running loop raises
                self._loop.close()

    async def _serve(self) -> Server:
        # serve() needs the running loop, so it is built on the loop thread.
        return await serve(self.manager.handle, "127.0.0.1", 0)

    def _launch(self, drone_id: str, index: int, url: str) -> None:
        start = from_local(DEFAULT_HOME, (index * DRONE_SPACING_M, 0.0, 0.0))
        drone = MockDrone(drone_id, start, DRONE_SPEED_M_S, REPORT_PERIOD_S)
        stop = threading.Event()
        run = asyncio.run_coroutine_threadsafe(drone.run(url, stop.is_set), self._loop)
        self._flyers[drone_id] = _Flyer(stop, run)

    def _photograph(self, state: DroneState) -> None:
        photo = photo_from_state(state)
        keep = self.keep_photo(photo)
        with self._photos_lock:
            self._photos.append(photo)
            if not keep:
                self._held.append(photo)
        if keep:
            self.photo_sink(photo)

    def _call(self, coroutine: Coroutine[Any, Any, T]) -> T:
        return asyncio.run_coroutine_threadsafe(coroutine, self._loop).result(
            timeout=STOP_TIMEOUT_S
        )


@contextmanager
def flying(drone_ids: Iterable[str]) -> Iterator[FleetHarness]:
    """
    A started ``FleetHarness`` with every drone reporting, closed on exit.
    """

    harness = FleetHarness(drone_ids)
    harness.start()
    try:
        harness.wait_until_reporting()
        yield harness
    finally:
        harness.close()


def _running_loop() -> asyncio.AbstractEventLoop | None:
    try:
        return asyncio.get_running_loop()
    except RuntimeError:
        return None
