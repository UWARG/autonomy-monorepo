"""
Target reconnaissance subtree: the engine commands nothing while the pilot
flies and captures images on manual triggers through the always-running
``triggered_image_publisher`` node. Waits for the recon-complete RC switch
before handing over to the land phase.

TargetReconnaissance
└── WaitForReconComplete
"""

from __future__ import annotations

import py_trees
from engine.behaviors.rc.rc_switch import WaitForRCSwitch
from engine.constants import RC_SWITCHES_ENABLED, RECON_COMPLETE_RC_CHANNEL


def create_target_reconnaissance_subtree() -> py_trees.behaviour.Behaviour:
    """Build the target reconnaissance subtree."""

    children: list[py_trees.behaviour.Behaviour] = []
    if RC_SWITCHES_ENABLED:
        children.append(
            WaitForRCSwitch(
                name="WaitForReconComplete",
                channel=RECON_COMPLETE_RC_CHANNEL,
            )
        )

    return py_trees.composites.Sequence(
        name="TargetReconnaissance",
        memory=True,
        children=children,
    )
