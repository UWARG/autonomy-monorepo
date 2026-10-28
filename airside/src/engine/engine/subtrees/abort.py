"""
Abort subtree: once ABORT is received from the ground, land immediately and
hold the tree there.

Abort
├── CheckAbort
├── RetryLand [Retry forever]
│   └── Land
└── Aborted [Running]
"""

from __future__ import annotations

import py_trees
from engine.behaviors.navigation.land import Land
from engine.behaviors.safety_supervisor import CheckAbort


def create_abort_subtree() -> py_trees.behaviour.Behaviour:
    """Build the abort subtree."""

    return py_trees.composites.Sequence(
        name="Abort",
        memory=True,
        children=[
            CheckAbort(),
            py_trees.decorators.Retry(
                name="RetryLand",
                child=Land(name="AbortLand"),
                num_failures=-1,
            ),
            py_trees.behaviours.Running(name="Aborted"),
        ],
    )