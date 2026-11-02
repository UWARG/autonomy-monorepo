from dataclasses import dataclass
from utils.src.types import Coordinate

@dataclass
class Mission:
    """Data class for missions for GNC module"""
    task_id : str 
    target : Coordinate
    yaw : float 