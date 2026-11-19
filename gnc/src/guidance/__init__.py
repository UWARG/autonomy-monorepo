"""Smooth viewpoint guidance and real-time tracking."""

from .models import (
    CapturePoint,
    GimbalModel,
    GimbalState,
    GuidanceLimits,
    ReferenceTrajectory,
    TrackerCommand,
    TrackerConfig,
    TrackerStatus,
    Viewpoint,
)
from .reference import build_reference
from .tracker import GuidanceTracker

__all__ = [
    "CapturePoint",
    "GimbalModel",
    "GimbalState",
    "GuidanceLimits",
    "GuidanceTracker",
    "ReferenceTrajectory",
    "TrackerCommand",
    "TrackerConfig",
    "TrackerStatus",
    "Viewpoint",
    "build_reference",
]
