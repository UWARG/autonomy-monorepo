"""
Land subtree: fly back to the home coordinate from the waypoints config,
then land and wait for disarm.

LandPhase
├── LoadHomeWaypoint
├── RetryFlyToHome [Retry forever]
│   └── FlyToHome (FlyToWaypoint)
└── Land
"""

from __future__ import annotations

import py_trees
from airside.src.engine.engine.behaviors.flight.land import Land
from airside.src.engine.engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from airside.src.engine.engine.behaviors.navigation.load_home_waypoint import LoadHomeWaypoint


def create_land_subtree() -> py_trees.behaviour.Behaviour:
    """Build the land subtree."""

    fly_to_home = py_trees.decorators.Retry(
        name="RetryFlyToHome",
        child=FlyToWaypoint(name="FlyToHome"),
        num_failures=-1,
    )

    return py_trees.composites.Sequence(
        name="LandPhase",
        memory=True,
        children=[
            LoadHomeWaypoint(),
            fly_to_home,
            Land(),
        ],
    )
