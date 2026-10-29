from __future__ import annotations

import math
import time
from threading import Thread, Event, Lock

from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .mission import Mission

from groundside_manager import DroneState as DroneState, DroneStateManager


class DroneTaskScheduler:
    def __init__(self, missions: list[Mission], drone_manager: DroneStateManager) -> None:
        self.POLLING_HERTZ = 0.5
        self.STALE_TIMESTAMP_LIMIT = 4

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
        while not self.stop_polling.is_set():

            try:
                available_drones = self.drone_manager.get_available_drones()
                removal_list = []

                unavailable_drones = set()

                for index, mission in enumerate(self.missions):
                    result = self.assign_mission(
                        mission, available_drones, unavailable_drones)

                    if result is not None:
                        removal_list.append(index)
                        unavailable_drones.add(result)

                """
                Won't break since it will always remove the indices from the largest
                to the smallest. This allows the smaller indices to remain it in the 
                right location when the larger indices get removed. 
                """
                with self.missions_list_lock:
                    for index in reversed(removal_list):
                        del self.missions[index]
            except Exception:
                # Ignore error, keep trying 
                pass
            finally:
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
        unavailable_drones: set[str],
    ) -> str | None:
        """
        Attempts to assign out the mission to the closest proximity drone. 

        If it successfully assigns out the mission to a drone, it returns drone_id. 
        Otherwise it returns None, for when it couldn't find a drone or the drone
        did not accept the mission. 
        """
        closest_drone = None
        closest_proximity = float('inf')

        for drone in available_drones:
            if (drone.drone_id in unavailable_drones
                    or drone.position is None or drone.last_update_timestamp is None):
                continue

            # Possibly stale position, ignore drone
            if time.time() - drone.last_update_timestamp >= self.STALE_TIMESTAMP_LIMIT:
                continue

            proximity = self.get_proximity(mission.target, drone.position)

            if proximity < closest_proximity:
                closest_proximity = proximity
                closest_drone = drone

        if closest_drone is not None:
            result = self.drone_manager.send_location_command_sync(
                closest_drone.drone_id,
                mission.task_id,
                mission.target,
                mission.yaw
            )

            # send_location_command_sync returns a boolean of whether it accepted or not
            if result:
                return closest_drone.drone_id

        return None
