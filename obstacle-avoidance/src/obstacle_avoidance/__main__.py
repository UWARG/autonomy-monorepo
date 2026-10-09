"""Minimal local demonstration; flight integration lives in airside."""

from __future__ import annotations

from .models import CircleObstacle, ObstacleSnapshot, PlanRequest, Point2D
from .planner import BendyRuler2D


def main() -> None:
    snapshot = ObstacleSnapshot(
        obstacles=(CircleObstacle(Point2D(5.0, 0.0), 1.5),),
        timestamp_s=0.0,
    )
    result = BendyRuler2D().plan(
        PlanRequest(
            start=Point2D(0.0, 0.0),
            goal=Point2D(15.0, 0.0),
            obstacles=snapshot,
            now_s=0.0,
        )
    )
    print(result)


if __name__ == "__main__":
    main()
