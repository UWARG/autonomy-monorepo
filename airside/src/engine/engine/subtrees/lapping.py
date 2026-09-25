"""Lapping subtree with obstacle-aware waypoint navigation."""

from __future__ import annotations

import py_trees

from engine.behaviors.navigation.lap_timing import (
    RecordLapEnd,
    ResetLapWaypoints,
    SetLappingDeadline,
    StartLapTimer,
)
from engine.behaviors.navigation.load_next_waypoint import LoadNextWaypoint
from engine.behaviors.navigation.obstacle_aware_fly_to_waypoint import (
    ObstacleAwareFlyToWaypoint,
)
from engine.behaviors.navigation.time_checks import (
    EnoughTimeForAnotherLap,
    EnoughTimeRemaining,
)


def create_lapping_subtree() -> py_trees.behaviour.Behaviour:
    """Build the lapping subtree."""
    single_waypoint = py_trees.composites.Sequence(
        name="SingleWaypoint",
        memory=True,
        children=[
            LoadNextWaypoint(),
            EnoughTimeRemaining(),
            ObstacleAwareFlyToWaypoint(),
            StartLapTimer(),
        ],
    )

    waypoint_loop = py_trees.decorators.FailureIsSuccess(
        name="UntilWaypointsExhausted",
        child=py_trees.decorators.Repeat(
            name="ForEachWaypoint",
            child=single_waypoint,
            num_success=-1,
        ),
    )

    single_lap = py_trees.composites.Sequence(
        name="SingleLap",
        memory=True,
        children=[
            EnoughTimeForAnotherLap(),
            ResetLapWaypoints(),
            waypoint_loop,
            RecordLapEnd(),
        ],
    )

    lap_until_deadline = py_trees.decorators.FailureIsSuccess(
        name="LapUntilDeadline",
        child=py_trees.decorators.Repeat(
            name="RepeatLap",
            child=single_lap,
            num_success=-1,
        ),
    )

    return py_trees.composites.Sequence(
        name="Lapping",
        memory=True,
        children=[
            SetLappingDeadline(),
            lap_until_deadline,
        ],
    )
