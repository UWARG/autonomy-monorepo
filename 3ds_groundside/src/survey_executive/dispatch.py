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

logger = logging.getLogger(__name__)

# Just under the scheduler's 4 s stale limit, so a drone that never picked up
# a command is retried before the rest of the system calls it stale.
DEFAULT_ACK_TIMEOUT_S = 3.0


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

    Not thread-safe: call every method from the one thread that runs ``tick``.
    """

    def __init__(
        self,
        manager: DroneManagerLike,
        clock: Callable[[], float],
        ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S,
    ) -> None:
        if ack_timeout_s <= 0:
            raise ValueError("ack_timeout_s must be positive")
        self._manager = manager
        self._clock = clock
        self._ack_timeout_s = ack_timeout_s
        self._queues: dict[str, list[Mission]] = {}
        self._flights: dict[str, _Flight] = {}
        self._completed: list[tuple[str, str]] = []
        self._task_ids: set[str] = set()

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
        for drone_id, route in routes.items():
            self._queues.setdefault(drone_id, []).extend(route)

    def tick(self) -> list[tuple[str, str]]:
        """
        Checks every drone once and sends what is due.

        Returns the ``(drone_id, task_id)`` pairs finished during this tick.
        Disconnected or unknown drones are skipped; their Missions wait.
        """

        states = self._manager.get_all_drones()
        finished: list[tuple[str, str]] = []
        for drone_id in self._queues:
            state = states.get(drone_id)
            if state is None or not state.connected:
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

    def is_done(self) -> bool:
        """
        True once every loaded Mission has finished.
        """

        return not self._flights and not any(self._queues.values())

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
