""" 
Enums used throughout the Airside system
"""

from enum import Enum 
from dataclasses import dataclass 

@dataclass 
class Colour: 
    """Data Class for Colour  """

class Colours(Enum): 
    """Enum for Colours featuring their HSV values"""

class Direction(Enum):
    """Enum for Directions"""

class MavlinkMessageType(Enum): 
    """Enum for Mavlink Message Types"""

class CameraType(Enum): 
    """Enum for Camera Types"""

class CurrentState(Enum): 
    """Enum for the current state of the drone"""
