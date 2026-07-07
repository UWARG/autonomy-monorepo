"""Enums used throughout the Airside system."""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple


@dataclass
class Colour:
    """Data class for colour values."""

    name: str
    lower_hsv: Tuple[int, int, int]
    upper_hsv: Tuple[int, int, int]

    def __str__(self) -> str:
        return f"({self.name}, {self.lower_hsv}, {self.upper_hsv})"

    def __repr__(self) -> str:
        return f"Colour(name={self.name}, lower_hsv={self.lower_hsv}, upper_hsv={self.upper_hsv})"


class Colours(Enum):
    """Enum for colours featuring their HSV ranges (H: 0-180, S: 0-255, V: 0-255)."""

    RED = Colour("Red", (0, 100, 100), (10, 255, 255))
    RED2 = Colour("Red", (168, 100, 100), (180, 255, 255))
    GREEN = Colour("Green", (40, 100, 100), (80, 255, 255))
    BLUE = Colour("Blue", (100, 100, 100), (130, 255, 255))
    YELLOW = Colour("Yellow", (20, 100, 100), (30, 255, 255))
    WHITE = Colour("White", (0, 0, 200), (180, 20, 255))
    BLACK = Colour("Black", (0, 0, 0), (180, 255, 20))


class Direction(Enum):
    """Enum for cardinal directions."""

    NORTH = "NORTH"
    SOUTH = "SOUTH"
    EAST = "EAST"
    WEST = "WEST"


class MavlinkMessageType(Enum):
    """Enum for MAVLink message types."""

    GLOBAL_POSITION_INT = "GLOBAL_POSITION_INT"
    ATTITUDE = "ATTITUDE"
    RC_CHANNELS = "RC_CHANNELS"


class CameraType(Enum):
    """Enum for camera device types."""

    OPENCV = 0
    PICAM2 = 1
    ARDUCAMIR = 2
    OAK_D = 3
