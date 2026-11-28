"""
Plans the next round of photos over the gaps coverage found.
"""

from __future__ import annotations

from shapely.geometry import Polygon
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState

from .planning import PlanningError, Waypoint, plan_waypoints, to_missions
from .request import SurveyRequest


def plan_reshoot(
    request: SurveyRequest,
    gaps_enu: list[Polygon],
    anchor: Coordinate,
    drones: list[DroneState],
    round_index: int,
) -> tuple[dict[str, list[Mission]], list[Polygon]]:
    """
    Plans every gap with the given drones; returns (routes, unplanned gaps).

    ``gaps_enu`` are in the survey ENU frame of ``anchor``, in the order to
    fly them. Each gap is planned over its outer edge: gnc rejects holes, and
    reshooting a small already-covered island inside a gap costs little.

    Each drone starts a gap where its route so far ends, or at its last
    known position before its first gap. A gap the planner rejects is
    returned in the second list instead of failing the whole round.
    """

    if round_index < 1:
        raise ValueError("reshoot round_index starts at 1")
    starts: dict[str, Coordinate | None] = {
        drone.drone_id: drone.position for drone in drones
    }
    if len(starts) != len(drones):
        raise ValueError("duplicate drone IDs")

    routes: dict[str, list[Waypoint]] = {drone_id: [] for drone_id in starts}
    unplanned: list[Polygon] = []
    for gap in gaps_enu:
        try:
            if not drones:
                raise PlanningError("no drones left")
            planned = plan_waypoints(request, dict(starts), _gap_boundary(gap, anchor))
        except PlanningError:
            unplanned.append(gap)
            continue
        for drone_id, waypoints in planned.items():
            routes[drone_id].extend(waypoints)
            if waypoints:
                starts[drone_id] = waypoints[-1][0]

    return to_missions(request.survey_id, round_index, routes), unplanned


def _gap_boundary(gap: Polygon, anchor: Coordinate) -> list[Coordinate]:
    """
    Outer edge of a gap as GPS points, without the repeated first point.
    """

    return [
        enu_to_coordinate(anchor, east_m, north_m)
        for east_m, north_m in list(gap.exterior.coords)[:-1]
    ]
