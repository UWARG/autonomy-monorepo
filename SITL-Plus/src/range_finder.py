"""PyBullet range finder sensor simulation and Rerun logging."""

import math
import os
import struct
import time

import numpy as np
import pybullet as p
import rerun as rr

import constants
import state

prev_state = state.update


def _sensor_host():
    host = os.getenv("SENSOR_HOST")
    if not host:
        raise ValueError("SENSOR_HOST environment variable is not set")
    return host


class Range_Finder:  # pylint: disable=invalid-name
    """Simulated downward range finder using PyBullet ray tests."""

    def __init__(self, port, direction=None, dist=100):
        if direction is None:
            direction = [0, 0, -1]
        self.port = port
        self.direction = direction
        self.dist = dist
        self.range = 0.0

    def update(self):
        """Cast a ray and store the distance to the nearest obstacle."""
        pos, orn = p.getBasePositionAndOrientation(state.robot_id)
        ray_from = np.array(pos)
        rot_matrix = p.getMatrixFromQuaternion(orn)
        rot_matrix = np.reshape(rot_matrix, (3, 3))
        local_direction = rot_matrix @ np.array(self.direction)
        local_direction /= np.linalg.norm(local_direction)
        ray_to = ray_from + local_direction * self.dist

        result = p.rayTest(ray_from, ray_to)
        body_id = result[0][0]
        coordinates = result[0][3]
        while body_id == state.robot_id:
            ray_from = ray_from + local_direction * 0.01
            result = p.rayTest(ray_from, ray_to)
            body_id = result[0][0]
            coordinates = result[0][3]
        self.range = math.sqrt(
            (coordinates[0] - ray_from[0]) ** 2
            + (coordinates[1] - ray_from[1]) ** 2
            + (coordinates[2] - ray_from[2]) ** 2
        )

    def range_thread(self):
        """Periodically update and publish range finder readings."""
        while True:
            time.sleep(1 / constants.RANGE_FINDER_FPS)
            self.update()
            state.airside_socket.sendto(
                struct.pack("f", self.range), (_sensor_host(), self.port)
            )
            rr.log(str(self.port) + "_range", rr.Scalars(self.range))
