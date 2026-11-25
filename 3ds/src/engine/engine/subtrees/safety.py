"""
Fence breach subtree: while the drone is breached or recovering, fly it to the
recovery target staged by ``CheckFenceBreach``.

FenceBreach [Sequence, memory=False]
├── CheckFenceBreach
└── RecoverIntoGeofence
"""

from __future__ import annotations

import py_trees

from engine.behaviors.safety_supervisor import (
    CheckFenceBreach,
    RecoverIntoGeofence,
)


def create_fence_breach_subtree() -> py_trees.behaviour.Behaviour:
    """Build the fence breach subtree."""

    return py_trees.composites.Sequence(
        name="FenceBreach",
        # Re-evaluate the fence every tick instead of remembering a SUCCESS.
        memory=False,
        children=[
            CheckFenceBreach(),
            RecoverIntoGeofence(),
        ],
    )
