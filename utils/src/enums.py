"""Enums used throughout the Airside system."""

from dataclasses import dataclass
from enum import Enum


@dataclass
class Colour:
    """Data class for colour values."""


class Colours(Enum):
    """Enum for colours featuring their HSV values."""


class Direction(Enum):
    """Enum for directions."""


class MavlinkMessageType(Enum):
    """Enum for MAVLink message types."""


class CameraType(Enum):
    """Enum for camera types."""