"""Deterministic two-segment local search for planar obstacle avoidance."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .clearance import CircularObstacleField, ClearanceField
from .geometry import angular_distance, distance, heading, project, wrap_angle
from .models import NoPathReason, PlanRequest, PlanResult, PlanStatus, Point2D


@dataclass(frozen=True, slots=True)
class PlannerConfig:
    first_lookahead_m: float = 8.0
    second_lookahead_m: float = 8.0
    clearance_margin_m: float = 1.0
    candidate_step_deg: float = 15.0
    maximum_detour_deg: float = 120.0
    map_freshness_s: float = 0.5
    goal_tolerance_m: float = 0.25
    heading_cost_m_per_rad: float = 0.35
    clearance_reward_weight: float = 0.05
    clearance_reward_cap_m: float = 5.0
    hysteresis_cost_m: float = 0.75

    def __post_init__(self) -> None:
        positive = (
            self.first_lookahead_m,
            self.second_lookahead_m,
            self.clearance_margin_m,
            self.candidate_step_deg,
            self.map_freshness_s,
            self.goal_tolerance_m,
            self.clearance_reward_cap_m,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("planner distances, steps, and freshness must be positive")
        if not 0.0 <= self.maximum_detour_deg <= 180.0:
            raise ValueError("maximum detour must be between 0 and 180 degrees")
        if self.heading_cost_m_per_rad < 0.0:
            raise ValueError("heading cost must be non-negative")
        if self.clearance_reward_weight < 0.0:
            raise ValueError("clearance reward must be non-negative")
        if self.hysteresis_cost_m < 0.0:
            raise ValueError("hysteresis cost must be non-negative")


@dataclass(frozen=True, slots=True)
class _Candidate:
    first: Point2D
    second: Point2D
    heading_rad: float
    clearance_m: float
    cost: float
    order: int


class BendyRuler2D:
    """Stateful local planner with only heading hysteresis as memory.

    The planner owns no vehicle, middleware, or sensor behavior. A caller may
    pass any object implementing ``ClearanceField``; circular obstacles are the
    default representation used by the package's sector adapter and tests.
    """

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        self._previous_heading_rad: float | None = None

    def reset(self) -> None:
        self._previous_heading_rad = None

    def plan(
        self,
        request: PlanRequest,
        clearance_field: ClearanceField | None = None,
    ) -> PlanResult:
        age_s = request.now_s - request.obstacles.timestamp_s
        if age_s < 0.0 or age_s > self.config.map_freshness_s:
            self.reset()
            return PlanResult.no_path(NoPathReason.STALE_MAP)
        if not request.obstacles.healthy:
            self.reset()
            return PlanResult.no_path(NoPathReason.UNHEALTHY_MAP)

        goal_distance = distance(request.start, request.goal)
        if goal_distance <= self.config.goal_tolerance_m:
            self.reset()
            return PlanResult(
                status=PlanStatus.PATH_FOUND,
                waypoint=request.goal,
                second_waypoint=request.goal,
                heading_rad=None,
                minimum_clearance_m=(
                    clearance_field
                    or CircularObstacleField.from_snapshot(request.obstacles)
                ).point_clearance(request.goal),
            )

        field = clearance_field or CircularObstacleField.from_snapshot(
            request.obstacles
        )
        goal_heading = heading(request.start, request.goal)
        candidates = self._feasible_candidates(request, field, goal_heading)
        if not candidates:
            self.reset()
            return PlanResult.no_path(NoPathReason.BLOCKED)

        best = min(candidates, key=self._sort_key)
        selected = self._apply_hysteresis(
            request=request,
            field=field,
            goal_heading=goal_heading,
            candidates=candidates,
            best=best,
        )
        self._previous_heading_rad = selected.heading_rad
        return PlanResult(
            status=PlanStatus.PATH_FOUND,
            waypoint=selected.first,
            second_waypoint=selected.second,
            heading_rad=selected.heading_rad,
            minimum_clearance_m=selected.clearance_m,
        )

    def _feasible_candidates(
        self,
        request: PlanRequest,
        field: ClearanceField,
        goal_heading: float,
    ) -> list[_Candidate]:
        candidates: list[_Candidate] = []
        for order, candidate_heading in enumerate(
            self._candidate_headings(goal_heading)
        ):
            candidate = self._evaluate(
                request,
                field,
                goal_heading,
                candidate_heading,
                order,
            )
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _candidate_headings(self, goal_heading: float) -> tuple[float, ...]:
        count = int(self.config.maximum_detour_deg // self.config.candidate_step_deg)
        headings = [goal_heading]
        for index in range(1, count + 1):
            offset = math.radians(index * self.config.candidate_step_deg)
            # Counter-clockwise is the documented deterministic symmetric tie.
            headings.extend(
                (wrap_angle(goal_heading + offset), wrap_angle(goal_heading - offset))
            )
        return tuple(headings)

    def _evaluate(
        self,
        request: PlanRequest,
        field: ClearanceField,
        goal_heading: float,
        candidate_heading: float,
        order: int,
    ) -> _Candidate | None:
        first_distance = min(
            self.config.first_lookahead_m,
            distance(request.start, request.goal),
        )
        first = project(request.start, candidate_heading, first_distance)
        first_clearance = field.segment_clearance(request.start, first)
        if first_clearance < self.config.clearance_margin_m:
            return None

        remaining = distance(first, request.goal)
        if remaining <= self.config.goal_tolerance_m:
            second = request.goal
            second_clearance = field.point_clearance(second)
        else:
            second = project(
                first,
                heading(first, request.goal),
                min(self.config.second_lookahead_m, remaining),
            )
            second_clearance = field.segment_clearance(first, second)
        clearance = min(first_clearance, second_clearance)
        if clearance < self.config.clearance_margin_m:
            return None

        capped_clearance = min(clearance, self.config.clearance_reward_cap_m)
        cost = (
            distance(second, request.goal)
            + self.config.heading_cost_m_per_rad
            * angular_distance(candidate_heading, goal_heading)
            - self.config.clearance_reward_weight * capped_clearance
        )
        return _Candidate(
            first=first,
            second=second,
            heading_rad=candidate_heading,
            clearance_m=clearance,
            cost=cost,
            order=order,
        )

    @staticmethod
    def _sort_key(candidate: _Candidate) -> tuple[float, int]:
        return (candidate.cost, candidate.order)

    def _apply_hysteresis(
        self,
        request: PlanRequest,
        field: ClearanceField,
        goal_heading: float,
        candidates: list[_Candidate],
        best: _Candidate,
    ) -> _Candidate:
        if self._previous_heading_rad is None:
            return best

        previous = min(
            candidates,
            key=lambda candidate: (
                angular_distance(candidate.heading_rad, self._previous_heading_rad),
                candidate.order,
            ),
        )
        exact_previous = self._evaluate(
            request,
            field,
            goal_heading,
            self._previous_heading_rad,
            previous.order,
        )
        if exact_previous is not None:
            previous = exact_previous
        if previous.cost <= best.cost + self.config.hysteresis_cost_m:
            return previous
        return best
