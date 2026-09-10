"""
Dropping subtree: release payload items onto unoccupied landing pads until no
items remain on board.

Dropping
├── LoadLandingPads
└── DropUntilEmpty [FailureIsSuccess]
    └── RepeatDrops [Repeat forever]
        └── SingleDrop
            ├── HoldingItem            (loop guard: while holding at least one item)
            ├── FindUnoccupiedLandingPad
            ├── DescendToLandingPad
            ├── TouchingGround
            ├── ReleasePayload
            └── AscendToAltitude
"""

from __future__ import annotations

import py_trees
from engine.behaviors.navigation.ascend_to_altitude import AscendToAltitude
from engine.behaviors.navigation.descend_to_pad import DescendToLandingPad
from engine.behaviors.navigation.landing_pads import (
    FindUnoccupiedLandingPad,
    HoldingItem,
    LoadLandingPads,
)
from engine.behaviors.navigation.touching_ground import TouchingGround
from engine.behaviors.payload.release_payload import ReleasePayload


def create_dropping_subtree() -> py_trees.behaviour.Behaviour:
    """Build the dropping subtree."""

    single_drop = py_trees.composites.Sequence(
        name="SingleDrop",
        memory=True,
        children=[
            HoldingItem(),
            FindUnoccupiedLandingPad(),
            DescendToLandingPad(),
            TouchingGround(),
            ReleasePayload(),
            AscendToAltitude(),
        ],
    )

    drop_loop = py_trees.decorators.FailureIsSuccess(
        name="DropUntilEmpty",
        child=py_trees.decorators.Repeat(
            name="RepeatDrops",
            child=single_drop,
            num_success=-1,
        ),
    )

    return py_trees.composites.Sequence(
        name="Dropping",
        memory=True,
        children=[
            LoadLandingPads(),
            drop_loop,
        ],
    )
