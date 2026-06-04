from dataclasses import dataclass
import math

import numpy as np


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class ImageFrame:
    u: float
    v: float


def _build_rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Build rotation matrix from Tait-Bryan angles."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    r_z = np.array([[cy, -sy, 0.0],
                    [sy,  cy, 0.0],
                    [0.0, 0.0, 1.0]])

    r_y = np.array([[ cp, 0.0, sp],
                    [0.0, 1.0, 0.0],
                    [-sp, 0.0, cp]])

    r_x = np.array([[1.0, 0.0,  0.0],
                    [0.0,  cr, -sr],
                    [0.0,  sr,  cr]])

    return r_z @ r_y @ r_x


def locate_target(
    frame: ImageFrame,
    intrinsics: CameraIntrinsics,
    altitude: float,
    yaw: float,
    pitch: float,
    roll: float,
) -> np.ndarray:
    """Return target position [north, east, down] relative to drone in NED (metres).

    altitude: perpendicular distance from drone to target plane.
    yaw/pitch/roll: drone attitude in radians (NED, ZYX Tait-Bryan convention).

    Camera frame is assumed aligned with the drone body.
    """
    d_cam = np.array([
        1.0,
        (frame.u - intrinsics.cx) / intrinsics.fx,
        (frame.v - intrinsics.cy) / intrinsics.fy,
    ])

    R = _build_rotation_matrix(yaw, pitch, roll)
    d_world = R @ d_cam

    t = altitude / d_world[2]
    return t * d_world
