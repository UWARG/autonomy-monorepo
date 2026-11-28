"""
Setpoints subtree: fly to each setpoint sent from the ground and photograph
it, until the go-home command arrives.

The go-home command preempts the mission at any point, including mid-flight
to a setpoint. Setpoints that fail (navigation timeout or photo failure) are
skipped so the drone keeps serving the next one.

UntilGoHome [Parallel SuccessOnOne]
├── WaitForGoHome
└── ServeSetpoints [Repeat forever]
    └── SkipFailedSetpoint [FailureIsSuccess]
        └── ServeSetpoint
            ├── WaitForSetpoint
            ├── FlyToSetpoint (FlyToWaypoint)
            └── TakePhoto
"""

from __future__ import annotations

import py_trees
from engine.behaviors.camera.take_photo import TakePhoto
from engine.behaviors.commands.wait_for_go_home import WaitForGoHome
from engine.behaviors.commands.wait_for_setpoint import WaitForSetpoint
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint


def create_setpoints_subtree() -> py_trees.behaviour.Behaviour:
    """Build the setpoints subtree."""

    serve_setpoint = py_trees.composites.Sequence(
        name="ServeSetpoint",
        memory=True,
        children=[
            WaitForSetpoint(),
            FlyToWaypoint(name="FlyToSetpoint"),
            TakePhoto(),
        ],
    )

    serve_setpoints = py_trees.decorators.Repeat(
        name="ServeSetpoints",
        child=py_trees.decorators.FailureIsSuccess(
            name="SkipFailedSetpoint",
            child=serve_setpoint,
        ),
        num_success=-1,
    )

    return py_trees.composites.Parallel(
        name="UntilGoHome",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[
            WaitForGoHome(),
            serve_setpoints,
        ],
    )
