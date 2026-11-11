from __future__ import annotations

import logging
import math
import time
from collections import Counter
from threading import Event, Lock, Thread

from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from groundside_manager import DroneState, DroneStateManager

from .mission import Mission

logger = logging.getLogger(__name__)


class DroneTaskScheduler:
    def __init__(self, missions: list[Mission], drone_manager: DroneStateManager) -> None:
        self.POLLING_HERTZ = 0.5
        self.STALE_TIMESTAMP_LIMIT = 4
        self.DRONE_FAILED_ASSIGN_ATTEMPT_LIMIT = 3
        self.NEXT_BEST_DRONE_PROXIMITY_LIMIT = 5

        self.missions = list(missions)
        self.missions_list_lock = Lock()

        self.drone_manager = drone_manager

        """
        If this is set, it means that the thread should NOT run.

        .set() will block the thread from running.
        .clear() will allow the thread to run.

        This was done this way so that when self.stop() is called to stop the thread,
        even if it's on the wait() section of the loop, it will stop the thread instead
        of waiting the remaining time. This is due to wait() ending immediately if another
        thread calls .set() to set the internal flag to true. 
        """
        self.stop_polling = Event()
        self.polling_thread = None

        self.stop_polling.set()  # Immediately set it so the thread is blocked from running

    def start(self) -> None:
        """
        Starts the thread to continuously poll get_available_drones() to 
        assign missions to drones.

        It will run self.stop() beforehand to stop any existing thread before
        starting a new one.
        """
        self.stop()

        self.polling_thread = Thread(
            target=self._poll
        )

        self.stop_polling.clear()
        self.polling_thread.start()

    def stop(self) -> None:
        """
        Stops the thread from continuously polling get_available_drones() and 
        calls .join() on the thread.
        """
        self.stop_polling.set()

        if self.polling_thread is not None:
            self.polling_thread.join()

    def _poll(self) -> None:
        """
        Internal method to continuously poll get_available_drones() and get any 
        available drones to assign the remaining missions to.

        Blocked if self.stop_polling.is_set() is True. See variable definition for
        more information.
        """

        # Counter to keep track of the number of failed attempts to assign
        # to a drone
        failed_assign_attempts = Counter()

        while not self.stop_polling.is_set():
            removal_list = []

            try:
                available_drones = self.drone_manager.get_available_drones()

                assigned_drones = set()

                for index, mission in enumerate(self.missions):
                    result = self.assign_mission(
                        mission, available_drones, assigned_drones,
                        failed_assign_attempts)

                    if result is not None:
                        removal_list.append(index)
                        assigned_drones.add(result)
            except Exception:
                # Ignore error, keep trying
                logger.exception(
                    "An error occurred while attempting to assign missions to available drones."
                )
            finally:
                """
                Won't break since it will always remove the indices from the largest
                to the smallest. This allows the smaller indices to remain it in the 
                right location when the larger indices get removed. 
                """
                with self.missions_list_lock:
                    for index in reversed(removal_list):
                        del self.missions[index]

                self.stop_polling.wait(1 / self.POLLING_HERTZ)

    def add_mission(self, new_mission: Mission) -> None:
        """
        Adds a mission to the list
        """
        with self.missions_list_lock:
            self.missions.append(new_mission)

    def get_proximity(self, target: Coordinate, drone_location: Coordinate) -> float:
        """
        Gets the proximity from the target to the drone_location
        """
        east, north = east_north_coordinate_offset_m(drone_location.lat, drone_location.lon,
                                                     target.lat, target.lon)

        height_difference = target.alt - drone_location.alt

        return math.hypot(east, north, height_difference)

    def assign_mission(
        self,
        mission: Mission,
        available_drones: list[DroneState],
        assigned_drones: set[str],
        failed_assign_attempts: Counter[str]
    ) -> str | None:
        """
        Assigns the mission to the nearest available drone that accepts it,
        falling back to the next best candidate when close enough or when the
        closer drone keeps rejecting.

        Returns the accepting drone_id, or None if no drone accepted it.
        """
        def acceptable_drone_filter(drone):
            # Keep only drones that are not already assigned this pass, have a
            # position, and have a non-stale timestamp.
            return (
                drone.drone_id not in assigned_drones
                and drone.position is not None
                and drone.last_update_timestamp is not None
                and time.time() - drone.last_update_timestamp
                < self.STALE_TIMESTAMP_LIMIT
            )

        def map_drone_output(drone):
            proximity = self.get_proximity(mission.target, drone.position)

            return (proximity, drone.drone_id)

        # Each element is a tuple that is defined as (proximity, drone_id)
        closest_drones_candidates = filter(
            acceptable_drone_filter, available_drones)
        closest_drones_candidates = sorted(
            map(map_drone_output, closest_drones_candidates))

        if len(closest_drones_candidates) != 0:
            for index, (proximity, drone_id) in enumerate(closest_drones_candidates):
                assigned_drone_result = self.drone_manager.send_location_command_sync(
                    drone_id,
                    mission.task_id,
                    mission.target,
                    mission.yaw
                )

                # send_location_command_sync returns a boolean of whether it accepted or not
                if assigned_drone_result:
                    failed_assign_attempts.pop(drone_id, None)
                    return drone_id
                else:
                    failed_assign_attempts[drone_id] += 1

                if ((index != len(closest_drones_candidates) - 1 and closest_drones_candidates[index + 1][0] - proximity <= self.NEXT_BEST_DRONE_PROXIMITY_LIMIT)
                        or (failed_assign_attempts[drone_id] > self.DRONE_FAILED_ASSIGN_ATTEMPT_LIMIT)):
                    continue

                break

        return None
