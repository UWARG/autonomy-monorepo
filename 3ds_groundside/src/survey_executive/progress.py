"""
Survey state and progress, as plain data for the dashboard (#213).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class SurveyState(Enum):
    """
    Where a survey is. COMPLETE and ABORTED are final.
    """

    PLANNING = "PLANNING"
    SURVEYING = "SURVEYING"
    RESHOOTING = "RESHOOTING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"

    @property
    def is_final(self) -> bool:
        return self in (SurveyState.COMPLETE, SurveyState.ABORTED)


class DroneActivity(Enum):
    """
    What one drone is doing in the survey.
    """

    FLYING = "FLYING"  # has a mission in flight
    IDLE = "IDLE"  # nothing in flight, waiting or finished
    DROPPED = "DROPPED"  # stopped reporting; its missions went to others


@dataclass(frozen=True)
class DroneProgress:
    drone_id: str
    activity: DroneActivity
    in_flight_task_id: str | None
    missions_done: int
    missions_remaining: int


@dataclass(frozen=True)
class SurveyProgress:
    """
    A snapshot of one survey. ``percent_covered`` counts only accepted
    photos; ``fully_covered`` means no gap above the minimum area is left.
    """

    survey_id: str
    state: SurveyState
    round_index: int
    percent_covered: float
    fully_covered: bool
    photos_received: int
    photos_rejected: int
    missions_done: int
    missions_total: int
    missions_unassigned: int
    gaps_remaining: int
    gaps_unplanned: int = 0  # gaps the planner refused in the last reshoot
    abort_reason: str | None = None
    drones: list[DroneProgress] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        """
        JSON-ready dict: enums become their string values.
        """

        data = asdict(self)
        data["state"] = self.state.value
        for drone in data["drones"]:
            drone["activity"] = drone["activity"].value
        return data
