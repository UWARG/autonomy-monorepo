""" 
Dataclasses used throughout the Airside System 
"""

from dataclasses import dataclass, field 
from typing import Optional 
from .enums import Colours, Direction 

@dataclass
class Coordinate: 
    """Data Class for a Coordinate in 3D space"""
    
@dataclass
class Vector3D: 
    """Data Class for a 3D Vector"""

@dataclass
class Quaternion: 
    """Data Class for a Quaternion"""

@dataclass
class Plane: 
    """Data Class for a Plane"""

@dataclass 
class Target: 
    """Data Class for a Target"""

@dataclass
class MappedTarget: 
    """Data Class for a Target that has been mapped to the world frame"""

@dataclass
class PositionMessage: 
    """Data Class for a Position Message"""

@dataclass
class AttitudeMessage: 
    """Data Class for an Attitude Message"""

