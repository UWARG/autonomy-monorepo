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

    rgb: np.ndarray

    depth: np.ndarray | None = None

    centre_depth: float = math.nan

    timestamp: float = field(default_factory=time.time)
    
