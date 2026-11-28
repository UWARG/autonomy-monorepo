"""
Turns planner setpoints into ordered Missions with stable task IDs.
"""

from __future__ import annotations

import math
from itertools import pairwise

from gnc.src.orthomosaic import DroneSpec, ScanRequest, plan_swarm_orthomosaic
from utils.src.coordinates import anchor_for_boundary, coordinate_to_enu
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState

from .request import SurveyRequest

# Segments within this angle of each other count as the same pass direction.
_AXIS_TOLERANCE_RAD = math.radians(10.0)
# Shorter segments carry no usable direction (e.g. repeated setpoints).
_MIN_SEGMENT_M = 1e-6


class PlanningError(Exception):
    """
    The planner could not produce routes for this boundary and fleet.
    """


def task_id(survey_id: str, round_index: int, drone_id: str, index: int) -> str:
    """
    Stable ID for one setpoint. It never changes, even if redistributed.
    """

    return f"{survey_id}-r{round_index}-{drone_id}-{index:03d}"


# A setpoint and the yaw to hold there, radians clockwise from north.
Waypoint = tuple[Coordinate, float]


def plan_routes(
    request: SurveyRequest,
    drones: list[DroneState],
    round_index: int,
    boundary: list[Coordinate] | None = None,
) -> dict[str, list[Mission]]:
    """
    Plans one round and returns each drone's Missions in flying order.

    ``boundary`` replaces the request's boundary, which is how reshoot rounds
    plan a gap. Drones start from their last known position when they have
    one. Every drone gets an entry, empty if the planner gave it no passes.

    Yaw is radians clockwise from north, facing along the survey pass.

    Raises ``ValueError`` for a negative round or duplicate drone IDs, and
    ``PlanningError`` if the planner rejects the area or fleet.
    """

    if round_index < 0:
        raise ValueError("round_index must be non-negative")
    area = request.boundary if boundary is None else boundary
    starts = {drone.drone_id: drone.position for drone in drones}
    if len(starts) != len(drones):
        # The dict would silently drop a duplicate the planner rejects.
        raise ValueError("duplicate drone IDs")
    try:
        waypoints = plan_waypoints(request, starts, area)
    except PlanningError:
        raise PlanningError(
            f"planner rejected round {round_index} of survey {request.survey_id!r}"
        ) from None
    return to_missions(request.survey_id, round_index, waypoints)


def plan_waypoints(
    request: SurveyRequest,
    starts: dict[str, Coordinate | None],
    boundary: list[Coordinate],
) -> dict[str, list[Waypoint]]:
    """
    Plans one area for the drones in ``starts`` (ID to optional start).

    Returns ordered waypoints per drone, in the order of ``starts``. Raises
    ``PlanningError`` if the planner rejects the area or fleet.
    """

    scan = ScanRequest(
        boundary=boundary,
        camera=request.camera,
        target_gsd_m=request.target_gsd_m,
        forward_overlap=request.forward_overlap,
        side_overlap=request.side_overlap,
    )
    specs = [DroneSpec(drone_id, start=start) for drone_id, start in starts.items()]
    ok, setpoints = plan_swarm_orthomosaic(scan, specs)
    if not ok or setpoints is None:
        raise PlanningError("planner rejected the area or fleet")

    anchor = anchor_for_boundary(boundary)
    enu_routes = {
        drone_id: [coordinate_to_enu(anchor, point) for point in route]
        for drone_id, route in setpoints.items()
    }
    axis_rad = _pass_axis_rad(list(enu_routes.values()))
    return {
        drone_id: list(
            zip(setpoints[drone_id], _route_yaws_rad(enu_routes[drone_id], axis_rad))
        )
        for drone_id in starts
    }


def to_missions(
    survey_id: str, round_index: int, waypoints: dict[str, list[Waypoint]]
) -> dict[str, list[Mission]]:
    """
    Numbers each drone's waypoints in flying order as Missions.
    """

    return {
        drone_id: [
            Mission(
                task_id=task_id(survey_id, round_index, drone_id, index),
                target=point,
                yaw=yaw_rad,
            )
            for index, (point, yaw_rad) in enumerate(route)
        ]
        for drone_id, route in waypoints.items()
    }


def _heading_rad(start: tuple[float, float], end: tuple[float, float]) -> float | None:
    """
    Heading from start to end, radians clockwise from north in [0, 2*pi).
    """

    east = end[0] - start[0]
    north = end[1] - start[1]
    if math.hypot(east, north) < _MIN_SEGMENT_M:
        return None
    return math.atan2(east, north) % (2.0 * math.pi)


def _axis_gap_rad(a: float, b: float) -> float:
    """
    Angle between two lines (direction ignored), in [0, pi/2].
    """

    gap = abs(a - b) % math.pi
    return min(gap, math.pi - gap)


def _pass_axis_rad(routes: list[list[tuple[float, float]]]) -> float | None:
    """
    Most common segment direction across all routes, in [0, pi).

    Passes are parallel and there is at most one turn between two passes, so
    the largest group of parallel segments is the pass direction. None if no
    route has a usable segment.
    """

    lines = [
        heading % math.pi
        for route in routes
        for start, end in pairwise(route)
        if (heading := _heading_rad(start, end)) is not None
    ]
    if not lines:
        return None
    best = max(
        lines,
        key=lambda candidate: sum(
            _axis_gap_rad(candidate, other) <= _AXIS_TOLERANCE_RAD for other in lines
        ),
    )
    group = [line for line in lines if _axis_gap_rad(best, line) <= _AXIS_TOLERANCE_RAD]
    # Mean of line directions: double the angles so 1 deg and 179 deg agree.
    sine = sum(math.sin(2.0 * line) for line in group)
    cosine = sum(math.cos(2.0 * line) for line in group)
    return (math.atan2(sine, cosine) / 2.0) % math.pi


def _route_yaws_rad(
    route: list[tuple[float, float]], axis_rad: float | None
) -> list[float]:
    """
    Yaw for each setpoint: along the pass axis, pointing the way it is flown.

    Uses the departing segment, else the arriving one, whichever lies on the
    axis, so the photos at each end of a pass don't turn to face the turn.
    """

    if axis_rad is None:
        return [0.0] * len(route)
    yaws: list[float] = []
    for index, point in enumerate(route):
        neighbours = []
        if index + 1 < len(route):
            neighbours.append(_heading_rad(point, route[index + 1]))
        if index > 0:
            neighbours.append(_heading_rad(route[index - 1], point))
        yaw = axis_rad
        for heading in neighbours:
            if heading is not None and _axis_gap_rad(heading, axis_rad) <= _AXIS_TOLERANCE_RAD:
                # Pick the end of the axis line closest to the flown heading.
                forward = abs(math.remainder(heading - axis_rad, 2.0 * math.pi))
                yaw = axis_rad if forward <= math.pi / 2.0 else axis_rad + math.pi
                break
        yaws.append(yaw % (2.0 * math.pi))
    return yaws
