"""The single shared PyBullet world."""

import math

import pybullet as p
import pybullet_data

from object import Object

GRAVITY_MSS = 9.80665


class Environment:
    """Own physics initialization and advance all vehicles together."""

    def __init__(self, rate_hz=800):
        self.time_step = 1.0 / rate_hz
        self.time_now = 0.0
        self.client = p.connect(p.DIRECT)
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -GRAVITY_MSS)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

    def step(self, drones):
        """Apply every drone's controls before advancing the world once."""
        for drone in drones:
            drone.vehicle.update(drone.pwm)
        p.stepSimulation()
        self.time_now += self.time_step

    def close(self):
        """Release the physics connection."""
        p.disconnect(self.client)

    def load_scene(self):
        """Load the existing demonstration props."""
        objects = [
            Object("r2d2.urdf", position=[4, 6, 0], orientation=[0, 0, math.pi / 2]),
            Object(
                "sphere_small.urdf",
                position=[2, 2, 0],
                orientation=[math.pi / 2, 0, 0],
                scale=5,
            ),
            Object(
                "barrel",
                position=[2, 2, 3],
                orientation=[0, 0, 0],
                scale=1,
                radius=0.5,
                height=1,
            ),
            Object(
                "sphere", position=[1, 1, 3], orientation=[0, 0, 0], scale=1, radius=0.5
            ),
            Object(
                "hoop",
                position=[-3, 1, 3],
                orientation=[math.pi / 2, 0, 0],
                scale=1,
                radius=1,
            ),
        ]
        for obj in objects:
            obj.initialize()
