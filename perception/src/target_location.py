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
    pass