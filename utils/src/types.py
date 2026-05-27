"""Dataclasses used throughout the Airside system."""

from dataclasses import dataclass


@dataclass
class Coordinate:
    """Data class for a coordinate in 3D space."""


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
    time_boot_ms: int = 0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0