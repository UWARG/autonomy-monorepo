"""Pure control helpers for obstacle-aware waypoint navigation."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from obstacle_avoidance import (
    BendyRuler2D,
    ObstacleSnapshot,
    PlannerConfig,
    PlanRequest,
    PlanStatus,
    Point2D,
    SectorScan,
)


@dataclass(frozen=True, slots=True)
class ObstacleNavigationConfig:
    """Runtime configuration for the airside planner adapter."""

    control_rate_hz: float = 10.0
    horizontal_speed_mps: float = 2.0
    vertical_speed_mps: float = 1.0
    obstacle_radius_m: float = 0.75
    scan_freshness_s: float = 0.3
    telemetry_freshness_s: float = 1.0
    goal_tolerance_m: float = 1.0
    future_stamp_tolerance_s: float = 0.05
    expected_scan_frame: str = "base_link"
    guided_mode: str = "GUIDED"

    def __post_init__(self) -> None:
        positive = (
            self.control_rate_hz,
            self.horizontal_speed_mps,
            self.vertical_speed_mps,
            self.obstacle_radius_m,
            self.scan_freshness_s,
            self.telemetry_freshness_s,
            self.goal_tolerance_m,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("navigation rates, distances, and timeouts must be positive")
        if self.future_stamp_tolerance_s < 0.0:
            raise ValueError("future stamp tolerance must be non-negative")
        if not self.expected_scan_frame:
            raise ValueError("expected scan frame must not be empty")


@dataclass(frozen=True, slots=True)
class ScanConversion:
    """Validated planner scan plus an optional fail-closed reason."""

    scan: SectorScan
    source_stamp_s: float
    reason: str | None


@dataclass(frozen=True, slots=True)
class NavigationGoal:
    """One local ENU waypoint with relative altitude."""

    east_m: float
    north_m: float
    relative_altitude_m: float


@dataclass(frozen=True, slots=True)
class NavigationTelemetry:
    """Latest vehicle state expressed in ROS ENU and global coordinates."""

    east_m: float
    north_m: float
    yaw_enu_rad: float
    latitude: float
    longitude: float
    relative_altitude_m: float
    armed: bool
    mode: str
    pose_received_s: float
    fix_received_s: float
    altitude_received_s: float
    state_received_s: float
    fresh_after_guided_entry: bool = True


@dataclass(frozen=True, slots=True)
class NavigationDecision:
    """One controller-cycle result."""

    should_publish: bool
    east_mps: float
    north_mps: float
    up_mps: float
    planner_status: str
    reason: str | None
    minimum_clearance_m: float | None
    goal_distance_m: float | None
    path_found_count: int
    hold_count: int
    valid_navigation_cycle: bool
    goal_reached: bool


@dataclass(slots=True)
class ActiveNavigationClock:
    """Accumulate only cycles in which valid navigation was possible."""

    elapsed_s: float = 0.0
    _last_s: float | None = None

    def reset(self, now_s: float) -> None:
        self.elapsed_s = 0.0
        self._last_s = now_s

    def advance(self, now_s: float, count: bool) -> float:
        if self._last_s is None:
            self._last_s = now_s
            return self.elapsed_s
        delta_s = max(0.0, now_s - self._last_s)
        self._last_s = now_s
        if count:
            self.elapsed_s += delta_s
        return self.elapsed_s


def prepare_sector_scan(
    *,
    ranges_m: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    frame_id: str,
    source_stamp_s: float,
    previous_source_stamp_s: float | None,
    now_ros_s: float,
    received_monotonic_s: float,
    config: ObstacleNavigationConfig,
) -> ScanConversion:
    """Validate a full-circle ROS scan and adapt it to planner sectors."""

    reason: str | None = None
    if frame_id != config.expected_scan_frame:
        reason = "INVALID_SCAN_FRAME"
    elif not ranges_m:
        reason = "EMPTY_SCAN"
    elif not math.isfinite(angle_min_rad):
        reason = "INVALID_SCAN_ANGLE"
    elif not math.isfinite(angle_increment_rad) or angle_increment_rad <= 0.0:
        reason = "INVALID_SCAN_INCREMENT"
    elif (
        not math.isfinite(range_min_m)
        or not math.isfinite(range_max_m)
        or range_min_m < 0.0
        or range_max_m <= range_min_m
    ):
        reason = "INVALID_SCAN_LIMITS"
    elif angle_increment_rad * len(ranges_m) < (
        2.0 * math.pi - angle_increment_rad * 0.5
    ):
        reason = "INCOMPLETE_SCAN_COVERAGE"
    elif not math.isfinite(source_stamp_s) or source_stamp_s <= 0.0:
        reason = "INVALID_SCAN_STAMP"
    elif (
        previous_source_stamp_s is not None
        and source_stamp_s <= previous_source_stamp_s
    ):
        reason = "FROZEN_SCAN"
    elif source_stamp_s - now_ros_s > config.future_stamp_tolerance_s:
        reason = "FUTURE_SCAN"
    elif now_ros_s - source_stamp_s > config.scan_freshness_s:
        reason = "STALE_SCAN"

    converted: list[float | None] = []
    invalid_range = False
    for raw_range in ranges_m:
        try:
            range_m = float(raw_range)
        except (TypeError, ValueError):
            invalid_range = True
            converted.append(None)
            continue
        if math.isinf(range_m) and range_m > 0.0:
            converted.append(None)
        elif (
            math.isfinite(range_m)
            and range_min_m <= range_m <= range_max_m
            and range_m > 0.0
        ):
            converted.append(range_m)
        else:
            invalid_range = True
            converted.append(None)
    if reason is None and invalid_range:
        reason = "INVALID_SCAN_RANGE"

    safe_angle_min = angle_min_rad if math.isfinite(angle_min_rad) else 0.0
    safe_increment = (
        angle_increment_rad
        if math.isfinite(angle_increment_rad) and angle_increment_rad > 0.0
        else 1.0
    )
    return ScanConversion(
        scan=SectorScan(
            ranges_m=tuple(converted),
            angle_offset_rad=safe_angle_min,
            angle_increment_rad=safe_increment,
            timestamp_s=received_monotonic_s,
            healthy=reason is None,
        ),
        source_stamp_s=source_stamp_s,
        reason=reason,
    )


class ObstacleAwareController:
    """Convert a safe local planner result into an ENU velocity command."""

    def __init__(
        self,
        config: ObstacleNavigationConfig | None = None,
        planner_config: PlannerConfig | None = None,
    ) -> None:
        self.config = config or ObstacleNavigationConfig()
        self.planner = BendyRuler2D(
            planner_config
            or PlannerConfig(
                first_lookahead_m=8.0,
                second_lookahead_m=8.0,
                clearance_margin_m=1.0,
                map_freshness_s=self.config.scan_freshness_s,
                hysteresis_cost_m=0.75,
            )
        )
        self.path_found_count = 0
        self.hold_count = 0

    def reset(self) -> None:
        self.planner.reset()
        self.path_found_count = 0
        self.hold_count = 0

    def step(
        self,
        *,
        now_s: float,
        goal: NavigationGoal,
        telemetry: NavigationTelemetry,
        obstacles: ObstacleSnapshot | None,
        scan_reason: str | None = None,
    ) -> NavigationDecision:
        """Return the command for one control cycle."""

        if not telemetry.armed:
            return self.release("DISARMED")
        if telemetry.mode != self.config.guided_mode:
            return self.release("PILOT_CONTROL")
        if not telemetry.fresh_after_guided_entry:
            return self.hold("WAITING_FOR_FRESH_DATA")

        telemetry_ages = (
            now_s - telemetry.pose_received_s,
            now_s - telemetry.fix_received_s,
            now_s - telemetry.altitude_received_s,
            now_s - telemetry.state_received_s,
        )
        if any(
            age_s < 0.0 or age_s > self.config.telemetry_freshness_s
            for age_s in telemetry_ages
        ):
            return self.hold("STALE_TELEMETRY")
        if obstacles is None:
            return self.hold("NO_SCAN")
        if scan_reason is not None:
            return self.hold(scan_reason)

        start = Point2D(telemetry.east_m, telemetry.north_m)
        local_goal = Point2D(goal.east_m, goal.north_m)
        east_offset_m = goal.east_m - telemetry.east_m
        north_offset_m = goal.north_m - telemetry.north_m
        altitude_error_m = goal.relative_altitude_m - telemetry.relative_altitude_m
        goal_distance_m = math.sqrt(
            east_offset_m**2 + north_offset_m**2 + altitude_error_m**2
        )

        result = self.planner.plan(
            PlanRequest(
                start=start,
                goal=local_goal,
                obstacles=obstacles,
                now_s=now_s,
            )
        )
        if result.status is PlanStatus.NO_PATH or result.waypoint is None:
            reason = result.reason.value if result.reason is not None else "NO_PATH"
            return self.hold(reason, goal_distance_m=goal_distance_m)

        self.path_found_count += 1
        if goal_distance_m <= self.config.goal_tolerance_m:
            return NavigationDecision(
                should_publish=True,
                east_mps=0.0,
                north_mps=0.0,
                up_mps=0.0,
                planner_status=PlanStatus.PATH_FOUND.value,
                reason=None,
                minimum_clearance_m=result.minimum_clearance_m,
                goal_distance_m=goal_distance_m,
                path_found_count=self.path_found_count,
                hold_count=self.hold_count,
                valid_navigation_cycle=True,
                goal_reached=True,
            )

        delta_east_m = result.waypoint.x - telemetry.east_m
        delta_north_m = result.waypoint.y - telemetry.north_m
        horizontal_distance_m = math.hypot(delta_east_m, delta_north_m)
        horizontal_speed_mps = min(
            self.config.horizontal_speed_mps, horizontal_distance_m
        )
        if horizontal_distance_m > 0.0:
            east_mps = horizontal_speed_mps * delta_east_m / horizontal_distance_m
            north_mps = horizontal_speed_mps * delta_north_m / horizontal_distance_m
        else:
            east_mps = 0.0
            north_mps = 0.0
        up_mps = max(
            -self.config.vertical_speed_mps,
            min(self.config.vertical_speed_mps, altitude_error_m),
        )
        return NavigationDecision(
            should_publish=True,
            east_mps=east_mps,
            north_mps=north_mps,
            up_mps=up_mps,
            planner_status=PlanStatus.PATH_FOUND.value,
            reason=None,
            minimum_clearance_m=result.minimum_clearance_m,
            goal_distance_m=goal_distance_m,
            path_found_count=self.path_found_count,
            hold_count=self.hold_count,
            valid_navigation_cycle=True,
            goal_reached=False,
        )

    def release(self, reason: str) -> NavigationDecision:
        """Release setpoint ownership without publishing, and reset planner state."""

        self.planner.reset()
        return NavigationDecision(
            should_publish=False,
            east_mps=0.0,
            north_mps=0.0,
            up_mps=0.0,
            planner_status="RELEASED",
            reason=reason,
            minimum_clearance_m=None,
            goal_distance_m=None,
            path_found_count=self.path_found_count,
            hold_count=self.hold_count,
            valid_navigation_cycle=False,
            goal_reached=False,
        )

    def hold(
        self, reason: str, *, goal_distance_m: float | None = None
    ) -> NavigationDecision:
        """Return a fail-closed zero-velocity decision."""

        self.hold_count += 1
        return NavigationDecision(
            should_publish=True,
            east_mps=0.0,
            north_mps=0.0,
            up_mps=0.0,
            planner_status=PlanStatus.NO_PATH.value,
            reason=reason,
            minimum_clearance_m=None,
            goal_distance_m=goal_distance_m,
            path_found_count=self.path_found_count,
            hold_count=self.hold_count,
            valid_navigation_cycle=False,
            goal_reached=False,
        )
