"""Dataclasses used throughout the Airside system."""

from __future__ import annotations
from dataclasses import dataclass
import math
from typing import List

from .enums import Colours, Direction

def checkInput(obj, types: list):
    if not isinstance(obj, tuple(types)):
        raise ValueError(f"Invalid object type. Got {type(obj).__name__}, expected one of {[t.__name__ for t in types]}")

@dataclass(frozen=True)
class Coordinate:
    """A latitude/longitude coordinate with relative altitude."""

    lat: float
    lon: float
    alt: float  # Relative altitude in meters

    def __str__(self) -> str:
        return f"({self.lat}, {self.lon}, {self.alt})"


@dataclass
class Vector3D:
    """Data class for a 3D vector."""
    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"

    def __add__(self, other: Vector3D) -> Vector3D:
        checkInput(other, [Vector3D, Coordinate])
        return Vector3D(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z
        )

    def __sub__(self, other: Vector3D) -> Vector3D:
        checkInput(other, [Vector3D, Coordinate])
        return Vector3D(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z
        )

    def __neg__(self) -> Vector3D:
        return Vector3D(-self.x, -self.y, -self.z)

    def __mul__(self, scalar: float) -> Vector3D:
        checkInput(scalar, [float, int])
        return Vector3D(
            self.x * scalar,
            self.y * scalar,
            self.z * scalar
        )

    def __rmul__(self, scalar: float) -> Vector3D:
        checkInput(scalar, [float, int])
        return self.__mul__(scalar)

    def __repr__(self) -> str:
        return f"Vector3D({self.x}, {self.y}, {self.z})"

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Vector3D:
        return self * (1 / self.norm())

    def to_pure_quaternion(self) -> Quaternion:
        return Quaternion(0, self.x, self.y, self.z)

    def cross(self, other: Vector3D) -> Vector3D:
        checkInput(other, [Vector3D])
        return Vector3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def dot(self, other: Vector3D) -> float:
        checkInput(other, [Vector3D])
        return (
            self.x * other.x +
            self.y * other.y +
            self.z * other.z
        )

@dataclass
class Quaternion:
    """Data class for a quaternion."""
    w: float
    x: float
    y: float
    z: float
    
    def __mul__(self, other: Quaternion|float ) -> Quaternion:
        checkInput(other, [Quaternion, float])
        
        if type(other) is float:
            return Quaternion(self.w * other, self.x * other, self.y * other, self.z * other) 

        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z

        return Quaternion(
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2,
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2,
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2,
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        )
    
    def __rmul__(self, other: Quaternion|float) -> Quaternion: 
        checkInput(other, [Quaternion, float])

        return self.__mul__(other)

    def __neg__(self) -> Quaternion:
        return Quaternion(-self.w, -self.x, -self.y, -self.z)

    def distance(self) -> float: # Norm 
        return math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def c(self) -> Quaternion: # Conjugate 
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self) -> Quaternion: 
        return self * (1/ self.distance())

    def isVector3D(self) -> bool: 
        return abs(self.w) <= 1e-5 

    def to_vector3d(self) -> Vector3D: 
        if self.isVector3D(): 
            return Vector3D(self.x, self.y, self.z)
        else: 
            raise ValueError("Quaternion is not a vector3D")       

class Rotation: 
    """Class for handling rotation"""

    def __str__(self) -> str:
        return f"(w: {self.w}, x: {self.x}, y: {self.y}, z: {self.z})"

    def to_array(self) -> List[float]:
        return [self.w, self.x, self.y, self.z]

    def __init__(self, w: float, x: float, y: float, z: float) -> None: 
        self.q = Quaternion(w, x, y, z)

    @staticmethod
    def from_vector3d(axis_of_rotation: Vector3D, angle: float) -> Rotation: 
        checkInput(axis_of_rotation, [Vector3D])
        checkInput(angle, [float|int])

        if axis_of_rotation == Vector3D(0, 0, 0):
            raise ValueError("Axis of rotation cannot be zero") 

        axis = axis_of_rotation 
        angle = angle  

        axis = axis * (1/ axis.norm())
        half = angle/2 

        return Rotation(
            math.cos(half),
            axis.x * math.sin(half),
            axis.y * math.sin(half),
            axis.z * math.sin(half)
        )

    def rotate(self, v: Vector3D|Quaternion) -> Vector3D|Quaternion: 
        checkInput(v, [Vector3D, Quaternion])

        if type(v) is Vector3D: 
            v_q = v.to_pure_quaternion()
        else: 
            v_q = v

        rotated_q = self.q * v_q * self.q.c() 

        if v is Vector3D: 
            return rotated_q.to_vector3d()
        else: 
            return rotated_q 
    
    def rotate_quaternion(self, v: Quaternion) -> Quaternion: 
        checkInput(v, [Quaternion])
        return self.q * v * self.q.c() 

    def rotate_vector3d(self, v: Vector3D) -> Vector3D: 
        checkInput(v, [Vector3D]) 

        return self.rotate_quaternion(v.to_pure_quaternion()).to_vector3d()


@dataclass
class Pose: 
    """Data class for a pose"""
    position: Vector3D
    orientation: Quaternion

    def convert_to_relative(self, other: Pose) -> Pose: 
        checkInput(other, [Pose])

        relative_orientation = self.orientation.c() * other.orientation 
        relative_position = self.orientation.c() * (other.position - self.position).to_pure_quaternion() * self.orientation
        
        return Pose(relative_position.to_vector3d(), relative_orientation) 
    
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
class Attitude:
    """Data class for attitude."""
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float

@dataclass
class RcChannelsMessage:
    """Data class for an RC channels message."""
