"""
Lapping subtree: fly waypoint laps until the deadline is reached, then return SUCCESS.

Lapping
├── LoadWaypointList
└── LapUntilDeadline [FailureIsSuccess]
    └── RepeatLap [Repeat forever]
        └── SingleLap
            ├── EnoughTimeForAnotherLap
            ├── ResetLapWaypoints
            ├── UntilWaypointsExhausted [FailureIsSuccess]
            │   └── ForEachWaypoint [Repeat forever]
            │       └── SingleWaypoint
            │           ├── LoadNextWaypoint
            │           ├── EnoughTimeRemaining
            │           ├── FlyToWaypoint
            │           └── StartLapTimer (first waypoint only)
            └── RecordLapEnd
"""

from __future__ import annotations

import py_trees
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.behaviors.navigation.lap_timing import (
    RecordLapEnd,
    ResetLapWaypoints,
    StartLapTimer,
)
from engine.behaviors.navigation.load_next_waypoint import LoadNextWaypoint
from engine.behaviors.navigation.load_waypoint_list import LoadWaypointList
from engine.behaviors.navigation.time_checks import EnoughTimeForAnotherLap, EnoughTimeRemaining


def create_lapping_subtree() -> py_trees.behaviour.Behaviour:
    """Build the lapping subtree."""
    single_waypoint = py_trees.composites.Sequence(
        name="SingleWaypoint",
        memory=True,
        children=[
            LoadNextWaypoint(),
            EnoughTimeRemaining(),
            FlyToWaypoint(),
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
            LoadWaypointList(),
            lap_until_deadline,
        ],
    )
