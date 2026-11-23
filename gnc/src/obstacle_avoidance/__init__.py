"""Traffic-aware obstacle avoidance (an original 2D BendyRuler-style planner)."""

from .avoidance import AvoidanceConfig, ObstacleAvoider
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
from .traffic import live_traffic_buffer_m, traffic_to_obstacles

__all__ = [
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
    "live_traffic_buffer_m",
    "traffic_to_obstacles",
]
