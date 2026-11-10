"""Dataclasses used throughout the Airside system."""

import math
from dataclasses import dataclass
from typing import List, Tuple

from .constants import MIN_VECTOR_NORM
from .enums import Colours, Direction


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

    def normalized(self) -> "Quaternion":
        """Return this quaternion scaled to unit length.

        Raises:
            ValueError: If any component is non-finite, or the quaternion is too
                short for its direction to mean anything.
        """
        components = self.to_array()
        if not all(math.isfinite(component) for component in components):
            raise ValueError(f"Quaternion must be finite, got {components}.")

        norm = math.sqrt(sum(component * component for component in components))
        if norm < MIN_VECTOR_NORM:
            raise ValueError("Quaternion must have non-zero norm.")

        return Quaternion(
            w=self.w / norm, x=self.x / norm, y=self.y / norm, z=self.z / norm
        )


@dataclass
class Plane:
    """Data class for a plane in 3D space defined by a normal vector and offset from origin."""

    normal: Vector3D
    offset: float

    def __str__(self) -> str:
        return f"(offset={self.offset}, normal={self.normal})"


@dataclass(frozen=True)
class ImageFrame:
    """Where a detector found a target within an image, in pixels.

    "Frame" here is the image's coordinate frame, not a video frame: this carries a
    detection's location within an image, never pixel data.
    """

    u: float
    v: float

    def __str__(self) -> str:
        return f"(u={self.u}, v={self.v})"

    def to_tuple(self) -> Tuple[float, float]:
        return (self.u, self.v)

    def validate(self) -> None:
        """Raise ``ValueError`` if this detection cannot be back-projected."""
        if not math.isfinite(self.u) or not math.isfinite(self.v):
            raise ValueError(f"Pixel must be finite, got (u={self.u}, v={self.v}).")


@dataclass(frozen=True)
class TargetPosition:
    """Where a target is, relative to the drone's body origin, in metres.

    Expressed in the Forward-Right-Down (FRD) body frame. Position only: a single ray
    to a detection centroid carries no information about how the target is *oriented*,
    so this deliberately is not a full pose.
    """

    forward_m: float
    right_m: float
    down_m: float

    def __str__(self) -> str:
        return f"(forward={self.forward_m}, right={self.right_m}, down={self.down_m})"

    def to_array(self) -> List[float]:
        """As a ``[forward, right, down]`` list."""
        return [self.forward_m, self.right_m, self.down_m]

    @property
    def range_m(self) -> float:
        """Straight-line distance from the drone's body origin to the target."""
        return math.hypot(self.forward_m, self.right_m, self.down_m)


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
