"""
Groundside Drone State Manager: tracks all identified drones and their latest known state.

SKELETON ONLY: types and method signatures; no logic implemented yet.
"""

import asyncio
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Any

# TODO: confirm this import path matches how other groundside modules import from utils
from utils.src.types import Attitude, Coordinate


class MissionState(Enum):
    """
    What a drone is currently doing. Values match the strings in DRONE_STATE.
    """

    IDLE = "IDLE"
    MOVING = "MOVING"
    # TODO: confirm the full list of states wuth PM


@dataclass
class DroneState:
    """
    Latest known state of one drone. Stored by the manager, sent out as copies.
    """
    drone_id: str
    vehicle_type: str # Pulled from REGISTER, e.g. "QUADCOPTER"
    position: Coordinate | None # None until the first DRONE_STATE packet arrives
    orientation: Attitude | None # None until the first DRONE_STATE packet arrives
    mission_state: MissionState
    active_task_id: str | None
    last_update_timestamp: float | None # when the last DRONE_STATE arrived
    connected: bool


class DroneStateManager:
    """
    Websocket server that registers drones and tracks their states.
    """
    def __init__(self):
        self._states: dict[str, DroneState] = {}
        self._connections: dict[str, Any] = {} # drone_id -> websocket (priv)
        self._lock = threading.Lock() # "guards" both dicts across threads to prevent conflicted writing/reading
        self._loop: asyncio.AbstractEventLoop | None = None # server's event loop

    # Getters: plain sync methods, safe to call from another thread. return copies.
    def get_drone(self, drone_id: str) -> DroneState | None:
        raise NotImplementedError

    def get_all_drones(self) -> dict[str, DroneState]:
        raise NotImplementedError

    def get_available_drones(self) -> list[DroneState]:
        """Connected drones with no active task"""
        raise NotImplementedError

    def is_connected(self, drone_id: str) -> bool:
        raise NotImplementedError

    # Sending commands (wait until the Task Scheduler #169 is done. Probably a seperate issue)
    async def send_location_command(
        self, drone_id: str, task_id: str, target: Coordinate, yaw: float
    ) -> bool:
        """Runs on the server's event loop. True if sent, False if drone not connected"""
        raise NotImplementedError

    def send_location_command_sync(
        self, drone_id: str, task_id: str, target: Coordinate, yaw: float
    ) -> bool:
        """For callers on another thread (e.g. the scheduler). Bridges to the loop."""
        raise NotImplementedError