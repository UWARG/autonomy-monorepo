"""
Fake drone fleet shared by the survey executive unit tests.
"""

from __future__ import annotations

from dataclasses import replace

from utils.src.types import Coordinate

from src.groundside_manager.groundside_manager import DroneState, MissionState
from tests.fakes import FakeClock
from tests.unit.survey_executive.factories import make_drone


class FakeFleet:
    """
    Drone states the test sets by hand, plus a log of every command sent.

    Connected drones that are not ``silent`` report on every read, so their
    ``last_update_timestamp`` is always the current time.
    ``send_location_command_sync`` returns ``connected``, like the planned
    #153 method: sent or not, never whether the drone took it.
    """

    def __init__(self, *drone_ids: str) -> None:
        self.clock = FakeClock()
        self.states = {drone_id: make_drone(drone_id) for drone_id in drone_ids}
        self.silent: set[str] = set()
        self.sent: list[tuple[str, str]] = []
        # The last command each drone was sent: (task_id, target, yaw).
        self.commands: dict[str, tuple[str, Coordinate, float]] = {}

    def get_all_drones(self) -> dict[str, DroneState]:
        for drone_id, state in self.states.items():
            if state.connected and drone_id not in self.silent:
                state.last_update_timestamp = self.clock()
        return {drone_id: replace(state) for drone_id, state in self.states.items()}

    def kill(self, drone_id: str) -> None:
        """
        The drone stops reporting and the link drops.
        """

        self.silent.add(drone_id)
        self.states[drone_id].connected = False

    def revive(self, drone_id: str) -> None:
        self.silent.discard(drone_id)
        self.states[drone_id].connected = True

    def send_location_command_sync(
        self, drone_id: str, task_id: str, target: Coordinate, yaw: float
    ) -> bool:
        state = self.states.get(drone_id)
        if state is None or not state.connected:
            return False
        self.sent.append((drone_id, task_id))
        self.commands[drone_id] = (task_id, target, yaw)
        return True

    def report(
        self, drone_id: str, mission_state: MissionState, task_id: str | None
    ) -> None:
        self.states[drone_id].mission_state = mission_state
        self.states[drone_id].active_task_id = task_id

    def fly(self, drone_id: str, task_id: str) -> None:
        """
        The drone takes the task and reaches it before the next tick.
        """

        self.report(drone_id, MissionState.ARRIVED, task_id)
