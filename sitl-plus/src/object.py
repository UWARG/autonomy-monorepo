"""PyBullet scene object loader."""

import logging
import sys

import pybullet as p


class Object:
    """Load URDF/SDF assets or primitive shapes into the PyBullet scene."""

    def __init__(
        self,
        name,
        position=None,
        orientation=None,
        scale=1,
        **kwargs,
    ):
        if position is None:
            position = [0, 0, 0]
        if orientation is None:
            orientation = [0, 0, 0]
        orientation = p.getQuaternionFromEuler(orientation)
        self.suffix = name.split(".")[-1]
        self.name = name
        self.position = position
        self.orientation = orientation
        self.scale = scale
        self.kwargs = kwargs
        self.id = None

    def _barrel_object(self):
        try:
            radius = self.kwargs["radius"]
            height = self.kwargs["height"]
        except KeyError:
            logging.error("Barrel object requires radius and length parameters")
            sys.exit(1)
        barrel_vision = p.createVisualShape(
            p.GEOM_CYLINDER, radius=radius, length=height
        )
        barrel_collision = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=radius, height=height
        )
        self.id = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=barrel_vision,
            baseCollisionShapeIndex=barrel_collision,
            basePosition=self.position,
            baseOrientation=self.orientation,
        )

    def _sphere_object(self):
        try:
            radius = self.kwargs["radius"]
        except KeyError:
            logging.error("Sphere object requires radius parameter")
            sys.exit(1)
        sphere_vision = p.createVisualShape(p.GEOM_SPHERE, radius=radius)
        sphere_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
        self.id = p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=sphere_vision,
            baseCollisionShapeIndex=sphere_collision,
            basePosition=self.position,
            baseOrientation=self.orientation,
        )

    def _hoop_object(self):
        try:
            radius = self.kwargs["radius"]
        except KeyError:
            logging.error("Hoop object requires radius parameter")
            sys.exit(1)
        scale = radius / 0.380861
        self.id = p.loadURDF(
            "hoop.urdf", self.position, self.orientation, globalScaling=scale
        )

    def initialize(self):
        """Create the object in the PyBullet world."""
        if self.suffix == "urdf":
            self.id = p.loadURDF(
                self.name, self.position, self.orientation, globalScaling=self.scale
            )
            return
        if self.suffix == "sdf":
            self.id = p.loadSDF(
                self.name, self.position, self.orientation, globalScaling=self.scale
            )
            return
        if self.name == "barrel":
            self._barrel_object()
            return
        if self.name == "sphere":
            self._sphere_object()
            return
        if self.name == "hoop":
            self._hoop_object()
            return
        logging.error("Unknown object type: %s", self.suffix)
        sys.exit(1)
