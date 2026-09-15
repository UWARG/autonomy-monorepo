"""
CameraFrame - Data class for each camera frame. 
"""

from dataclasses import dataclass, field
import time 
import math 
import numpy as np 

@dataclass 
class CameraFrame: 
    """Data Class for a Camera Frame"""

    # Colour image in RGB channel order (red, green, blue), shape (H, W, 3).
    rgb: np.ndarray

    depth: np.ndarray | None 

    rgb_down: np.ndarray | None

    downwards_range: float = math.nan 

    centre_depth: float = math.nan

    pitch: float = math.nan

    roll: float = math.nan

    timestamp: float = field(default_factory=time.time)
    
