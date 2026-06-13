"""Dataclasses used throughout the Airside system."""

from dataclasses import dataclass
from typing import List

from .enums import Colours, Direction


@dataclass
class Coordinate:
    """Data class for a coordinate in 3D space."""

    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"


@dataclass
class Vector3D:
    """Data class for a 3D vector."""

    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"


@dataclass
class Quaternion:
    """Data class for a quaternion."""

    w: float
    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"(w: {self.w}, x: {self.x}, y: {self.y}, z: {self.z})"

    def to_array(self) -> List[float]:
        return [self.w, self.x, self.y, self.z]


@dataclass
class Plane:
    """Data class for a plane in 3D space defined by a normal vector and offset from origin."""

    normal: Vector3D
    offset: float

    def __str__(self) -> str:
        return f"(offset={self.offset}, normal={self.normal})"


@dataclass
class Target:
    """Data class for a target."""

    colour: Colours
    location: Coordinate

    def __str__(self) -> str:
        return f"{self.colour.name}, {self.location}"


@dataclass
class MappedTarget:
    """Data class for a target mapped to the world frame."""

    colour: Colours
    location: Coordinate
    direction: Direction
    wall_target: bool = True

    def __str__(self) -> str:
        return (
            f"(colour={self.colour}, location={self.location}, "
            f"cardinal_direction={self.direction}, wall_target={self.wall_target})"
        )


@dataclass
class PositionMessage:
    """Data class for a position message received from MAVLink GLOBAL_POSITION_INT."""

    lat: float
    lon: float
    alt: float


@dataclass
class AttitudeMessage:
    """Data class for an attitude message received from MAVLink ATTITUDE."""

    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float
