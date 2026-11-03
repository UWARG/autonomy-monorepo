"""Lapping subtree: fly the waypoint list for the configured number of laps."""

from __future__ import annotations

import py_trees

from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.constants import LAP_WAYPOINT_ACCEPTANCE_RADIUS_M
from engine.behaviors.navigation.lap_count import AnotherLapRemaining, SetLapCount
from engine.behaviors.navigation.lap_timing import (
    RecordLapEnd,
    RecordWaypointReached,
    ResetLapWaypoints,
)
from engine.behaviors.navigation.load_next_waypoint import LoadNextWaypoint


def create_lapping_subtree() -> py_trees.behaviour.Behaviour:
    """Build the lapping subtree."""
    single_waypoint = py_trees.composites.Sequence(
        name="SingleWaypoint",
        memory=True,
        children=[
            LoadNextWaypoint(),
            FlyToWaypoint(
                acceptance_radius_m=LAP_WAYPOINT_ACCEPTANCE_RADIUS_M,
                fly_through=True,
            ),
            RecordWaypointReached(),
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
            AnotherLapRemaining(),
            ResetLapWaypoints(),
            waypoint_loop,
            RecordLapEnd(),
        ],
    )

    lap_until_done = py_trees.decorators.FailureIsSuccess(
        name="LapUntilDone",
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
            SetLapCount(),
            lap_until_done,
        ],
    )
