"""PyBullet scene object loader."""

import logging
import math
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pybullet as p


def create_landing_pad(
    center=(0.0, 0.0),
    size=10.0,
    texture_resolution=512,
    seed=42,
):
    """Create a non-repeating textured pad under home for ORB features.

    Returns the pad body id.
    """
    rng = np.random.default_rng(seed)
    tex = rng.integers(40, 220, (texture_resolution, texture_resolution, 3), dtype=np.uint8)
    # Multi-scale blobs so ORB finds stable keypoints from altitude.
    for _ in range(80):
        x = int(rng.integers(0, texture_resolution))
        y = int(rng.integers(0, texture_resolution))
        radius = int(rng.integers(8, 48))
        color = tuple(int(c) for c in rng.integers(0, 256, 3))
        cv2.circle(tex, (x, y), radius, color, thickness=-1)
    for _ in range(40):
        pt1 = (
            int(rng.integers(0, texture_resolution)),
            int(rng.integers(0, texture_resolution)),
        )
        pt2 = (
            int(rng.integers(0, texture_resolution)),
            int(rng.integers(0, texture_resolution)),
        )
        color = tuple(int(c) for c in rng.integers(0, 256, 3))
        cv2.line(tex, pt1, pt2, color, thickness=int(rng.integers(2, 8)))

    tex_path = Path(tempfile.gettempdir()) / "sitl_landing_pad.png"
    cv2.imwrite(str(tex_path), tex)

    half = size / 2.0
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[half, half, 0.005],
        rgbaColor=[1, 1, 1, 1],
    )
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[half, half, 0.005],
    )
    pad_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[center[0], center[1], 0.005],
    )
    tex_id = p.loadTexture(str(tex_path))
    p.changeVisualShape(pad_id, -1, textureUniqueId=tex_id)
    logging.info("Landing pad created at %s (size=%.1fm)", center, size)
    return pad_id


def create_landing_clutter(center=(0.0, 0.0)):
    """Scatter ducks and colored boxes near home for dense unique features."""
    cx, cy = center
    clutter_ids = []

    duck_poses = [
        (cx + 0.8, cy + 0.4, 0.0, 0.6),
        (cx - 1.2, cy + 0.9, 0.0, 0.8),
        (cx + 1.5, cy - 0.7, 0.0, 0.5),
        (cx - 0.5, cy - 1.4, 0.0, 0.7),
        (cx + 0.2, cy + 1.6, 0.0, 0.55),
        (cx - 1.8, cy - 0.3, 0.0, 0.65),
    ]
    for x, y, z, scale in duck_poses:
        yaw = math.atan2(y - cy, x - cx)
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        try:
            duck_id = p.loadURDF(
                "duck_vhacd.urdf",
                [x, y, z],
                orn,
                globalScaling=scale,
                useFixedBase=True,
            )
            clutter_ids.append(duck_id)
        except Exception as exc:  # noqa: BLE001 — pybullet raises generic errors
            logging.warning("Could not load duck_vhacd.urdf: %s", exc)
            break

    box_specs = [
        (cx + 0.3, cy - 0.5, 0.15, [0.2, 0.15, 0.15], [0.9, 0.2, 0.1, 1]),
        (cx - 0.9, cy + 0.2, 0.1, [0.25, 0.1, 0.1], [0.1, 0.7, 0.9, 1]),
        (cx + 1.1, cy + 1.0, 0.12, [0.15, 0.2, 0.12], [0.95, 0.85, 0.1, 1]),
        (cx - 1.4, cy - 1.0, 0.18, [0.18, 0.18, 0.18], [0.3, 0.9, 0.3, 1]),
        (cx + 0.0, cy + 0.9, 0.08, [0.3, 0.08, 0.08], [0.8, 0.4, 0.9, 1]),
        (cx - 0.2, cy - 1.8, 0.14, [0.12, 0.22, 0.14], [0.2, 0.2, 0.9, 1]),
    ]
    for x, y, z, half_extents, rgba in box_specs:
        visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba
        )
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        box_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[x, y, z],
        )
        clutter_ids.append(box_id)

    logging.info("Landing clutter placed: %d bodies", len(clutter_ids))
    return clutter_ids


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
