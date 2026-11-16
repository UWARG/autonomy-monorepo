"""
Setpoints subtree: fly to each setpoint sent from the ground and photograph
it, until the go-home command arrives.

The go-home command preempts the mission at any point. A rejected or
timed-out photo is retried at the same waypoint up to
``PHOTO_CAPTURE_MAX_ATTEMPTS`` times (``RetryPhoto``, wrapping only the
photo so a retry does not re-fly); only then does the sequence fail and
``SkipFailedSetpoint`` restarts it for the next setpoint.

UntilGoHome [Parallel SuccessOnOne]
├── WaitForGoHome
└── ServeSetpoints [Repeat forever]
    └── SkipFailedSetpoint [FailureIsSuccess]
        └── ServeSetpoint
            ├── WaitForSetpoint
            ├── FlyToSetpoint (FlyToWaypoint)
            └── RetryPhoto [Retry PHOTO_CAPTURE_MAX_ATTEMPTS]
                └── TakePhoto
"""

from __future__ import annotations

import py_trees
from engine.behaviors.camera.take_photo import TakePhoto
from engine.behaviors.commands.wait_for_go_home import WaitForGoHome
from engine.behaviors.commands.wait_for_setpoint import WaitForSetpoint
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.constants import PHOTO_CAPTURE_MAX_ATTEMPTS


def create_setpoints_subtree() -> py_trees.behaviour.Behaviour:
    """Build the setpoints subtree."""

    # Wraps only TakePhoto so a retry re-shoots without re-flying.
    take_photo = py_trees.decorators.Retry(
        name="RetryPhoto",
        child=TakePhoto(),
        num_failures=PHOTO_CAPTURE_MAX_ATTEMPTS,
    )

    serve_setpoint = py_trees.composites.Sequence(
        name="ServeSetpoint",
        memory=True,
        children=[
            WaitForSetpoint(),
            FlyToWaypoint(name="FlyToSetpoint"),
            take_photo,
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
