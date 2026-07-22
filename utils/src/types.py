"""Dataclasses used throughout the Airside system.

Two families live here:

* the **data / message** types -- ``Coordinate``, ``Plane``, ``Target``,
  ``MappedTarget``, ``Attitude``, ``RcChannelsMessage``;
* the **geometry / math kernel** -- ``Vector3D``, ``Quaternion``, ``Rotation``,
  ``Pose`` -- the vector algebra the gnc follow stack (frames, controller,
  simulation) is built on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .enums import Colours, Direction


def _check_input(obj, types: list) -> None:
    """Raise ``ValueError`` unless ``obj`` is an instance of one of ``types``."""
    if not isinstance(obj, tuple(types)):
        raise ValueError(
            f"Invalid object type. Got {type(obj).__name__}, "
            f"expected one of {[t.__name__ for t in types]}"
        )


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
    """A 3D vector with full vector algebra (operators, cross, dot, norm)."""

    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"

    def __repr__(self) -> str:
        return f"Vector3D({self.x}, {self.y}, {self.z})"

    def __add__(self, other: Vector3D) -> Vector3D:
        _check_input(other, [Vector3D])
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3D) -> Vector3D:
        _check_input(other, [Vector3D])
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self) -> Vector3D:
        return Vector3D(-self.x, -self.y, -self.z)

    def __mul__(self, scalar: float) -> Vector3D:
        _check_input(scalar, [float, int])
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> Vector3D:
        return self.__mul__(scalar)

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Vector3D:
        return self * (1 / self.norm())

    def cast_to_quaternion(self) -> Quaternion:
        return Quaternion(0, self.x, self.y, self.z)

    def cross(self, other: Vector3D) -> Vector3D:
        _check_input(other, [Vector3D])
        return Vector3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def dot(self, other: Vector3D) -> float:
        _check_input(other, [Vector3D])
        return self.x * other.x + self.y * other.y + self.z * other.z


@dataclass
class Quaternion:
    """Hamilton-convention quaternion, scalar-first ``(w, x, y, z)``."""

    w: float
    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"(w: {self.w}, x: {self.x}, y: {self.y}, z: {self.z})"

    def __mul__(self, other: Quaternion | float) -> Quaternion:
        _check_input(other, [Quaternion, float, int])
        if isinstance(other, (int, float)):
            return Quaternion(self.w * other, self.x * other, self.y * other, self.z * other)

        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def __rmul__(self, other: Quaternion | float) -> Quaternion:
        return self.__mul__(other)

    def __neg__(self) -> Quaternion:
        return Quaternion(-self.w, -self.x, -self.y, -self.z)

    def norm(self) -> float:
        """Scalar magnitude of the quaternion."""
        return math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Quaternion:
        """Unit quaternion in the same direction."""
        return self * (1 / self.norm())

    def c(self) -> Quaternion:
        """Conjugate ``(w, -x, -y, -z)``."""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def to_array(self) -> List[float]:
        return [self.w, self.x, self.y, self.z]

    def isVector3D(self, tol: float = 1e-9) -> bool:
        """True when the quaternion is (numerically) a pure vector (``w ~= 0``).

        Uses a tolerance rather than exact ``w == 0`` so that the residue left by
        a rotation sandwich product (``q v q*`` yields ``w`` of order 1e-16..1e-20)
        still counts as a vector -- otherwise ``rotate_vector3d`` and
        ``Pose.convert_to_relative`` would raise on perfectly valid rotations.
        """
        return abs(self.w) < tol

    def cast_to_vector3d(self) -> Vector3D:
        if self.isVector3D():
            return Vector3D(self.x, self.y, self.z)
        raise ValueError("Quaternion is not a pure vector (w != 0)")


class Rotation:
    """Axis-angle rotation, stored as a unit quaternion ``q``."""

    def __init__(self, axis_of_rotation: Vector3D, angle: float) -> None:
        _check_input(axis_of_rotation, [Vector3D])
        _check_input(angle, [float, int])
        if axis_of_rotation == Vector3D(0, 0, 0):
            raise ValueError("Axis of rotation cannot be zero")

        self.axis = axis_of_rotation * (1 / axis_of_rotation.norm())
        self.angle = angle
        half = angle / 2
        self.q = Quaternion(
            math.cos(half),
            self.axis.x * math.sin(half),
            self.axis.y * math.sin(half),
            self.axis.z * math.sin(half),
        )

    def rotate(self, v: Vector3D | Quaternion) -> Vector3D | Quaternion:
        """Rotate a vector or quaternion; dispatches on the runtime type."""
        _check_input(v, [Vector3D, Quaternion])
        if isinstance(v, Vector3D):
            return self.rotate_vector3d(v)
        return self.rotate_quaternion(v)

    def rotate_quaternion(self, v: Quaternion) -> Quaternion:
        _check_input(v, [Quaternion])
        return self.q * v * self.q.c()

    def rotate_vector3d(self, v: Vector3D) -> Vector3D:
        _check_input(v, [Vector3D])
        return self.rotate_quaternion(v.cast_to_quaternion()).cast_to_vector3d()


@dataclass
class Pose:
    """Rigid-body pose: a position and an orientation."""

    position: Vector3D
    orientation: Quaternion

    def convert_to_relative(self, other: Pose) -> Pose:
        """Express ``other`` in this pose's local frame."""
        _check_input(other, [Pose])
        relative_orientation = self.orientation.c() * other.orientation
        relative_position = (
            self.orientation.c()
            * (other.position - self.position).cast_to_quaternion()
            * self.orientation
        )
        return Pose(relative_position.cast_to_vector3d(), relative_orientation)


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
