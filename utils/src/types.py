"""Dataclasses used throughout the Airside system."""

from __future__ import annotations
from dataclasses import dataclass
import math

def checkInput(obj, types: list):
    if not isinstance(obj, tuple(types)):
        raise ValueError(f"Invalid object type. Got {type(obj).__name__}, expected one of {[t.__name__ for t in types]}")

@dataclass
class Coordinate:
    """Data class for a coordinate in 3D space."""

@dataclass
class Vector3D:
    """Data class for a 3D vector."""
    x: float
    y: float
    z: float

    def __add__(self, other: Vector3D) -> Vector3D:
        checkInput(other, [Vector3D, Coordinate])
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3D) -> Vector3D:
        checkInput(other, [Vector3D, Coordinate])
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self) -> Vector3D: 
        return Vector3D(-self.x, -self.y, -self.z)

    def __mul__(self, scalar: float) -> Vector3D: 
        checkInput(scalar, [float, int])
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __rmul__(self, scalar: float) -> Vector3D: 
        checkInput(scalar, [float, int])
        return self.__mul__(scalar)
        
    def __repr__(self) -> str:
        return f"Coordinate({self.x}, {self.y}, {self.z})"

    def norm(self) -> float: 
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalized(self)-> Vector3D:
        return self * (1 / self.norm())

    def cast_to_quaternion(self) -> Quaternion:
        return Quaternion(0, self.x, self.y, self.z)

    def cross(self, other: Vector3D): 
        checkInput(other, [Vector3D])
        return Vector3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def dot(self, other: Vector3D) -> float: 
        checkInput(other, [Vector3D])
        return self.x * other.x + self.y * other.y + self.z * other.z
        
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

    def norm(self) -> float: #norm 
        return math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def c(self) -> Quaternion: #Conjugate 
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self) -> Quaternion: 
        return self * (1/ self.norm())

    def isVector3D(self) -> bool: 
        return self.w == 0 

    def cast_to_vector3d(self) -> Vector3D: 
        if self.isVector3D() == True: 
            return Vector3D(self.x, self.y, self.z)
        else: 
            raise ValueError("Quaternion is not a vector3D")       

class Rotation: 
    """Class for handling rotation"""
    def __init__(self, axis_of_rotation: Vector3D, angle: float) -> None: 
        checkInput(axis_of_rotation, [Vector3D])
        checkInput(angle, [float|int])

        if axis_of_rotation == Vector3D(0, 0, 0):
            raise ValueError("Axis of rotation cannot be zero") 

        self.axis = axis_of_rotation 
        self.angle = angle  

        self.axis = self.axis * (1/ self.axis.norm())
        half = angle/2 

        self.q = Quaternion(
            math.cos(half),
            self.axis.x * math.sin(half),
            self.axis.y * math.sin(half),
            self.axis.z * math.sin(half)
        )

    def rotate(self, v: Vector3D|Quaternion) -> Vector3D|Quaternion: 
        checkInput(v, [Vector3D, Quaternion])

        if type(v) is Vector3D: 
            v_q = v.cast_to_quaternion()
        else: 
            v_q = v

        rotated_q = self.q * v_q * self.q.c() 

        if v is Vector3D: 
            return rotated_q.cast_to_vector3d()
        else: 
            return rotated_q 
    
    def rotate_quaternion(self, v: Quaternion) -> Quaternion: 
        checkInput(v, [Quaternion])
        return self.q * v * self.q.c() 

    def rotate_vector3d(self, v: Vector3D) -> Vector3D: 
        checkInput(v, [Vector3D]) 

        return self.rotate_quaternion(v.cast_to_quaternion()).cast_to_vector3d()


@dataclass
class Pose: 
    """Data class for a pose"""
    position: Vector3D
    orientation: Quaternion

    def convert_to_relative(self, other: Pose) -> Pose: 
        checkInput(other, [Pose])

        relative_orientation = self.orientation.c() * other.orientation 
        relative_position = self.orientation.c() * (other.position - self.position).cast_to_quaternion() * self.orientation
        
        return Pose(relative_position.cast_to_vector3d(), relative_orientation) 
    
@dataclass
class Plane:
    """Data class for a plane."""


@dataclass
class Target:
    """Data class for a target."""


@dataclass
class MappedTarget:
    """Data class for a target mapped to the world frame."""


@dataclass
class PositionMessage:
    """Data class for a position message."""


@dataclass
class AttitudeMessage:
    """Data class for an attitude message."""