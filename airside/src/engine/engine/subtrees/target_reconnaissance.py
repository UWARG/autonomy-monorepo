"""
Target reconnaissance subtree: passes the drone to the pilot for manual target
scanning, waits for the recon-complete RC switch, then triggers groundside
post-processing.

TargetReconnaissance
├── WaitForReconComplete
└── TriggerPostProcessing
"""

from __future__ import annotations

import py_trees
from engine.behaviors.rc.rc_switch import WaitForRCSwitch
from engine.behaviors.trigger_post_processing import TriggerPostProcessing
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
    children.append(TriggerPostProcessing())

    return py_trees.composites.Sequence(
        name="TargetReconnaissance",
        memory=True,
        children=children,
    )
