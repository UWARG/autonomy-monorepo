"""
Land subtree: fly back to the launch point (where the drone was armed), then
land and wait for disarm.

LandPhase
├── LoadLaunchPoint
├── RetryFlyToLaunchPoint [Retry forever]
│   └── FlyToLaunchPoint (FlyToWaypoint)
└── Land
"""

from __future__ import annotations

import py_trees
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.behaviors.navigation.land import Land
from engine.behaviors.navigation.load_launch_point import LoadLaunchPoint


def create_land_subtree() -> py_trees.behaviour.Behaviour:
    """Build the land subtree."""

    fly_to_launch_point = py_trees.decorators.Retry(
        name="RetryFlyToLaunchPoint",
        child=FlyToWaypoint(name="FlyToLaunchPoint"),
        num_failures=-1,
    )

    return py_trees.composites.Sequence(
        name="LandPhase",
        memory=True,
        children=[
            LoadLaunchPoint(),
            fly_to_launch_point,
            Land(),
        ],
    )
