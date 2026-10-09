"""A single mock drone connected to groundside over a websocket."""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import MutableMapping
from dataclasses import dataclass
from typing import Any, Callable

from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import InvalidURI, WebSocketException

from src.geometry import Vector3
from src.kinematics import step_toward
from src.messages import (
    LocationCommand,
    MissionState,
    command_ack,
    drone_state,
    encode,
    parse_location_command,
    register,
)

logger = logging.getLogger(__name__)

RECONNECT_MIN_DELAY_S = 1.0
RECONNECT_MAX_DELAY_S = 10.0


class _DroneLogAdapter(logging.LoggerAdapter):
    """Prefixes every log line with the drone's ID."""

    def process(
        self, msg: Any, kwargs: MutableMapping[str, Any]
    ) -> tuple[Any, MutableMapping[str, Any]]:
        assert self.extra is not None
        return f"{self.extra['drone_id']}: {msg}", kwargs


@dataclass(frozen=True)
class _Leg:
    """A straight-line flight from `start_position` at `start_time` to `target`."""

    start_position: Vector3
    start_time: float
    target: Vector3


class MockDrone:
    """A simulated drone that flies in a straight line to commanded targets.

    States cycle IDLE -> MOVING -> ARRIVED (for one update) -> IDLE.
    """

    def __init__(
        self,
        drone_id: str,
        start_position: Vector3,
        speed: float,
        report_period: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """`speed` is in m/s. `clock` drives the simulated flight only; reports are
        always sent every `report_period` real seconds."""
        for name, value in (("speed", speed), ("report_period", report_period)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"{name} must be a positive finite number, got {value}"
                )
        if not all(math.isfinite(v) for v in start_position):
            raise ValueError(f"start_position must be finite, got {start_position}")

        self._drone_id = drone_id
        self._speed = speed
        self._report_period = report_period
        self._clock = clock
        self._log = _DroneLogAdapter(logger, {"drone_id": drone_id})

        self._position = start_position
        self._command: LocationCommand | None = None
        self._mission_state = MissionState.IDLE
        self._leg: _Leg | None = None

    @property
    def drone_id(self) -> str:
        return self._drone_id

    @property
    def position(self) -> Vector3:
        return self._position

    @property
    def mission_state(self) -> MissionState:
        return self._mission_state

    @property
    def task_id(self) -> str | None:
        return self._command.task_id if self._command else None

    def handle_command(self, command: LocationCommand) -> bool:
        """Return whether the command is accepted. Only IDLE drones take new tasks; an
        exact resend of the last command is re-accepted without restarting."""
        if self._command is not None and command.task_id == self._command.task_id:
            if command == self._command:
                self._log.info("re-acknowledging %s", command.task_id)
                return True
            self._log.warning(
                "rejecting %s: task ID reused with a different target or yaw",
                command.task_id,
            )
            return False
        if self._mission_state is not MissionState.IDLE:
            self._log.info(
                "rejecting %s, still %s on %s",
                command.task_id,
                self._mission_state.value,
                self.task_id,
            )
            return False

        self._log.info("accepted %s, flying to %s", command.task_id, command.target)
        self._command = command
        self._leg = _Leg(
            start_position=self._position,
            start_time=self._clock(),
            target=command.target,
        )
        self._mission_state = MissionState.MOVING
        return True

    def update(self) -> None:
        """Recompute position and mission state from the clock."""
        if self._mission_state is MissionState.ARRIVED:
            self._mission_state = MissionState.IDLE
            self._leg = None
        elif self._leg is not None:
            elapsed = self._clock() - self._leg.start_time
            self._position = step_toward(
                self._leg.start_position, self._leg.target, self._speed, elapsed
            )
            if self._position == self._leg.target:
                self._log.info("arrived for %s", self.task_id)
                self._mission_state = MissionState.ARRIVED

    def state_message(self) -> dict[str, Any]:
        """The DRONE_STATE message describing the drone right now."""
        return drone_state(
            self._drone_id,
            self.task_id,
            self._mission_state,
            self._position,
            (0.0, 0.0, self._command.yaw if self._command else 0.0),
        )

    async def run(self, url: str, should_stop: Callable[[], bool]) -> None:
        """Run until `should_stop()`, reconnecting with backoff. Stopping can take up to
        one `report_period`. A malformed `url` raises InvalidURI."""
        delay = RECONNECT_MIN_DELAY_S
        while not should_stop():
            try:
                async with connect(url) as websocket:
                    delay = RECONNECT_MIN_DELAY_S
                    await self._run_session(websocket, should_stop)
            except InvalidURI:
                raise
            # asyncio.TimeoutError is only an OSError from Python 3.11 onwards.
            except (OSError, asyncio.TimeoutError, WebSocketException) as error:
                if should_stop():
                    return
                self._log.warning(
                    "connection to %s lost (%r), retrying in %.1fs", url, error, delay
                )
            else:
                if should_stop():
                    return
                self._log.warning(
                    "groundside closed the connection, retrying in %.1fs", delay
                )

            await _sleep_unless_stopped(delay, should_stop)
            delay = min(delay * 2, RECONNECT_MAX_DELAY_S)

    async def _run_session(
        self, websocket: ClientConnection, should_stop: Callable[[], bool]
    ) -> None:
        """Register, then handle commands and send state until either side stops."""
        await websocket.send(encode(register(self._drone_id)))
        self._log.info("registered")

        tasks = [
            asyncio.ensure_future(self._receive_loop(websocket)),
            asyncio.ensure_future(self._report_loop(websocket, should_stop)),
        ]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        # Re-raise a dropped connection so run() can reconnect.
        for task in done:
            task.result()

    async def _receive_loop(self, websocket: ClientConnection) -> None:
        """Handle LOCATION_COMMANDs until groundside closes the connection."""
        async for raw in websocket:
            command = parse_location_command(raw, self._drone_id)
            if command is None:
                continue
            accepted = self.handle_command(command)
            await websocket.send(
                encode(command_ack(self._drone_id, command.task_id, accepted))
            )

    async def _report_loop(
        self, websocket: ClientConnection, should_stop: Callable[[], bool]
    ) -> None:
        """Send an up-to-date DRONE_STATE once per report period."""
        deadline = time.monotonic()
        while not should_stop():
            self.update()
            await websocket.send(encode(self.state_message()))
            # Step from the last deadline so the rate doesn't drift, but restart from
            # now if we fell behind (e.g. a pause) rather than sending a burst.
            deadline = max(deadline + self._report_period, time.monotonic())
            await asyncio.sleep(deadline - time.monotonic())


async def _sleep_unless_stopped(
    seconds: float, should_stop: Callable[[], bool]
) -> None:
    """Sleep for `seconds`, waking early if `should_stop()` becomes True."""
    end = time.monotonic() + seconds
    while not should_stop():
        remaining = end - time.monotonic()
        if remaining <= 0:
            return
        await asyncio.sleep(min(remaining, 0.1))
