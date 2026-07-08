"""Dataclasses used throughout the Airside system."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Coordinate:
    """A latitude/longitude coordinate with relative altitude."""

    lat: float
    lon: float
    alt: float  # Relative altitude in meters


@dataclass
class Vector3D:
    """Data class for a 3D vector."""


@dataclass
class Quaternion:
    """Data class for a quaternion."""


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