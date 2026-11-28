"""
Sends each drone its planned Missions one at a time, in planned order.

The executive feeds per-drone queues directly instead of going through
``DroneTaskScheduler``, which gives each target to the nearest drone and would
scramble the planned routes.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState, MissionState

from .redistribute import split_among

logger = logging.getLogger(__name__)

# Just under the scheduler's 4 s stale limit, so a drone that never picked up
# a command is retried before the rest of the system calls it stale.
DEFAULT_ACK_TIMEOUT_S = 3.0
# Long enough that a short websocket reconnect doesn't reshuffle routes.
DEFAULT_DROPOUT_AFTER_S = 10.0


class DroneManagerLike(Protocol):
    """
    The part of ``DroneStateManager`` (#153) the executive uses.
    """

    def get_all_drones(self) -> dict[str, DroneState]: ...

    def send_location_command_sync(
        self, drone_id: str, task_id: str, target: Coordinate, yaw: float
    ) -> bool: ...


@dataclass
class _Flight:
    """
    The Mission a drone was last sent, and when it was last (re)sent or seen
    reporting it.
    """

    mission: Mission
    last_seen: float


class RouteDispatcher:
    """
    Keeps an ordered queue per drone and sends the next Mission only after the
    drone reports ARRIVED on the previous one.

    ``send_location_command_sync`` only says whether the command was sent, not
    whether the drone took it: a MOVING drone turns it down over the link. So
    the command is resent unchanged whenever the drone has gone
    ``ack_timeout_s`` without reporting its ``task_id`` (the mock drone
    re-accepts an exact resend without restarting the leg).

    A drone that sends no state update for ``dropout_after_s`` is dropped:
    its unfinished Missions, including the one in flight, are split among
    the drones still flying (see ``split_among``). If none are left they
    wait in a pool until one is. A dropped drone that reports again rejoins
    with an empty queue and only gets Missions still in the pool.

    A drone counts as reporting on a tick where it is connected and its
    ``last_update_timestamp`` has changed since the last tick. Only the
    change is used, never the value, so a skewed manager clock can't keep a
    dead drone alive; ``clock`` should be monotonic.
    Not thread-safe: call every method from the one thread that runs ``tick``.
    """

    def __init__(
        self,
        manager: DroneManagerLike,
        clock: Callable[[], float],
        ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S,
        dropout_after_s: float = DEFAULT_DROPOUT_AFTER_S,
    ) -> None:
        if ack_timeout_s <= 0:
            raise ValueError("ack_timeout_s must be positive")
        if dropout_after_s <= 0:
            raise ValueError("dropout_after_s must be positive")
        self._manager = manager
        self._clock = clock
        self._ack_timeout_s = ack_timeout_s
        self._dropout_after_s = dropout_after_s
        self._queues: dict[str, list[Mission]] = {}
        self._flights: dict[str, _Flight] = {}
        self._completed: list[tuple[str, str]] = []
        self._task_ids: set[str] = set()
        # When each drone last reported (or was first loaded, if it hasn't).
        self._seen_at: dict[str, float] = {}
        # The last timestamp read for each drone, to spot new reports.
        self._last_stamp: dict[str, float | None] = {}
        self._dropped: set[str] = set()
        # Unassigned runs of Missions, each still in planned order.
        self._pool: list[list[Mission]] = []

    def load(self, routes: dict[str, list[Mission]]) -> None:
        """
        Appends each drone's Missions after the ones it already has queued.

        Raises ``ValueError`` if a task ID repeats one already loaded, before
        anything is queued.
        """

        new_ids = [mission.task_id for route in routes.values() for mission in route]
        repeated = self._task_ids.intersection(new_ids)
        if repeated or len(set(new_ids)) != len(new_ids):
            # A repeated ID would make one ARRIVED report finish two Missions.
            raise ValueError(f"duplicate task IDs: {sorted(repeated) or new_ids}")
        self._task_ids.update(new_ids)
        now = self._clock()
        for drone_id, route in routes.items():
            self._seen_at.setdefault(drone_id, now)
            queue = self._queues.setdefault(drone_id, [])
            if drone_id in self._dropped:
                if route:
                    self._pool.append(list(route))
            else:
                queue.extend(route)

    def tick(self) -> list[tuple[str, str]]:
        """
        Checks every drone once and sends what is due.

        Returns the ``(drone_id, task_id)`` pairs finished during this tick.
        Disconnected or unknown drones are skipped until they are dropped.
        """

        now = self._clock()
        states = self._manager.get_all_drones()
        for drone_id in self._queues:
            reported = self._new_report(drone_id, states.get(drone_id))
            if reported:
                self._seen_at[drone_id] = now
            if drone_id in self._dropped:
                if reported:
                    logger.info("%s is reporting again, rejoining", drone_id)
                    self._dropped.discard(drone_id)
            elif now - self._seen_at[drone_id] >= self._dropout_after_s:
                self._drop(drone_id)
        self._reassign(states)

        finished: list[tuple[str, str]] = []
        for drone_id in self._queues:
            state = states.get(drone_id)
            if drone_id in self._dropped or state is None or not state.connected:
                continue
            done = self._step(drone_id, state)
            if done is not None:
                finished.append((drone_id, done))
        self._completed.extend(finished)
        return finished

    def completed(self) -> list[tuple[str, str]]:
        """
        Every ``(drone_id, task_id)`` finished so far, in the order finished.
        """

        return list(self._completed)

    def in_flight(self, drone_id: str) -> Mission | None:
        """
        The Mission the drone was sent and has not finished yet, if any.
        """

        flight = self._flights.get(drone_id)
        return None if flight is None else flight.mission

    def remaining(self, drone_id: str) -> list[Mission]:
        """
        The drone's unfinished Missions in order, the in-flight one first.
        """

        flight = self._flights.get(drone_id)
        head = [] if flight is None else [flight.mission]
        return head + list(self._queues.get(drone_id, []))

    def dropped(self) -> set[str]:
        """
        Drones currently dropped out.
        """

        return set(self._dropped)

    def unassigned(self) -> list[Mission]:
        """
        Missions waiting for a drone because none was left to take them.
        """

        return [mission for run in self._pool for mission in run]

    def is_done(self) -> bool:
        """
        True once every loaded Mission has finished.
        """

        return not self._flights and not self._pool and not any(self._queues.values())

    def _new_report(self, drone_id: str, state: DroneState | None) -> bool:
        """
        True if the drone is connected and sent an update since the last tick.

        The first read of a connected drone counts: if it is in fact dead, it
        is dropped ``dropout_after_s`` later.
        """

        stamp = None if state is None else state.last_update_timestamp
        seen_before = drone_id in self._last_stamp
        previous = self._last_stamp.get(drone_id)
        self._last_stamp[drone_id] = stamp
        return (
            state is not None
            and state.connected
            and stamp is not None
            and (not seen_before or stamp != previous)
        )

    def _drop(self, drone_id: str) -> None:
        run = self.remaining(drone_id)
        logger.warning(
            "%s dropped out with %d unfinished missions", drone_id, len(run)
        )
        self._dropped.add(drone_id)
        self._flights.pop(drone_id, None)
        self._queues[drone_id] = []
        if run:
            self._pool.append(run)

    def _reassign(self, states: dict[str, DroneState]) -> None:
        """
        Hands pooled Missions to the drones that can fly now.
        """

        survivors = [
            drone_id
            for drone_id in self._queues
            if drone_id not in self._dropped
            and (state := states.get(drone_id)) is not None
            and state.connected
        ]
        if not self._pool or not survivors:
            return
        for run in self._pool:
            # Recomputed per run, so later runs see the earlier chunks.
            places = {
                drone_id: self._end_place(drone_id, states[drone_id])
                for drone_id in survivors
            }
            for drone_id, chunk in split_among(run, places).items():
                logger.info(
                    "%s takes over %d missions starting at %s",
                    drone_id,
                    len(chunk),
                    chunk[0].task_id,
                )
                self._queues[drone_id].extend(chunk)
        self._pool = []

    def _end_place(self, drone_id: str, state: DroneState) -> Coordinate | None:
        """
        Where the drone will be once its current queue is flown.
        """

        queue = self._queues[drone_id]
        if queue:
            return queue[-1].target
        flight = self._flights.get(drone_id)
        if flight is not None:
            return flight.mission.target
        return state.position

    def _step(self, drone_id: str, state: DroneState) -> str | None:
        """
        Advances one drone. Returns the task ID it finished, if any.
        """

        now = self._clock()
        finished = None
        flight = self._flights.get(drone_id)
        if flight is not None:
            ours = state.active_task_id == flight.mission.task_id
            if ours and state.mission_state is MissionState.ARRIVED:
                finished = flight.mission.task_id
                del self._flights[drone_id]
                flight = None
            elif ours:
                flight.last_seen = now
                return None
            else:
                # Never taken, or lost after (e.g. the drone restarted).
                if now - flight.last_seen >= self._ack_timeout_s:
                    logger.info(
                        "%s is not on %s, resending",
                        drone_id,
                        flight.mission.task_id,
                    )
                    if self._send(drone_id, flight.mission):
                        flight.last_seen = now
                return None

        queue = self._queues[drone_id]
        free = state.mission_state in (MissionState.IDLE, MissionState.ARRIVED)
        # Send right after a finish too: the drone is ARRIVED on that task.
        if queue and free and self._send(drone_id, queue[0]):
            self._flights[drone_id] = _Flight(queue.pop(0), last_seen=now)
        return finished

    def _send(self, drone_id: str, mission: Mission) -> bool:
        sent = self._manager.send_location_command_sync(
            drone_id, mission.task_id, mission.target, mission.yaw
        )
        if not sent:
            logger.warning("could not send %s to %s", mission.task_id, drone_id)
        return sent
