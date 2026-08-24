"""Adapter from generic planar range sectors to conservative obstacles."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .models import CircleObstacle, ObstacleSnapshot, Point2D


@dataclass(frozen=True, slots=True)
class SectorScan:
    """Planar ranges relative to sensor heading; None means no return."""

    ranges_m: tuple[float | None, ...]
    angle_offset_rad: float
    angle_increment_rad: float
    timestamp_s: float
    healthy: bool = True


def sector_scan_to_snapshot(
    scan: SectorScan,
    sensor_position: Point2D,
    sensor_heading_rad: float,
    obstacle_radius_m: float,
) -> ObstacleSnapshot:
    """Project ranges; invalid values mark the snapshot unhealthy."""

    if obstacle_radius_m < 0.0:
        raise ValueError("obstacle radius must be non-negative")

    obstacles: list[CircleObstacle] = []
    healthy = scan.healthy
    for index, range_m in enumerate(scan.ranges_m):
        if range_m is None:
            continue
        if not math.isfinite(range_m) or range_m <= 0.0:
            healthy = False
            continue
        angle = (
            sensor_heading_rad
            + scan.angle_offset_rad
            + index * scan.angle_increment_rad
        )
        obstacles.append(
            CircleObstacle(
                center=Point2D(
                    sensor_position.x + math.cos(angle) * range_m,
                    sensor_position.y + math.sin(angle) * range_m,
                ),
                radius_m=obstacle_radius_m,
            )
        )

    return ObstacleSnapshot(
        obstacles=tuple(obstacles),
        timestamp_s=scan.timestamp_s,
        healthy=healthy,
    )
