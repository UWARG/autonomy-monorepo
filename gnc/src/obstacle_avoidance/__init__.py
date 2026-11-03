"""Traffic-aware obstacle avoidance (an original 2D BendyRuler-style planner)."""

from .avoidance import ESCAPE_REASON, AvoidanceConfig, ObstacleAvoider
from .clearance import CircularObstacleField, ClearanceField
from .models import (
    AvoidancePlan,
    AvoidanceRequest,
    CircleObstacle,
    NoPathReason,
    ObstacleSnapshot,
    PlanRequest,
    PlanResult,
    PlanStatus,
    Point2D,
    TrafficAircraft,
)
from .planner import BendyRuler2D, PlannerConfig
from .traffic import traffic_to_obstacles

__all__ = [
    "ESCAPE_REASON",
    "AvoidanceConfig",
    "AvoidancePlan",
    "AvoidanceRequest",
    "BendyRuler2D",
    "CircleObstacle",
    "CircularObstacleField",
    "ClearanceField",
    "NoPathReason",
    "ObstacleAvoider",
    "ObstacleSnapshot",
    "PlanRequest",
    "PlanResult",
    "PlanStatus",
    "PlannerConfig",
    "Point2D",
    "TrafficAircraft",
    "traffic_to_obstacles",
]
