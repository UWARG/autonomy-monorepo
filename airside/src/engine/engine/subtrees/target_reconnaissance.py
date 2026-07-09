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
from airside.src.engine.engine.behaviors.rc.rc_switch import WaitForRCSwitch
from airside.src.engine.engine.behaviors.trigger_post_processing import TriggerPostProcessing
from engine.constants import RECON_COMPLETE_RC_CHANNEL


def create_target_reconnaissance_subtree() -> py_trees.behaviour.Behaviour:
    """Build the target reconnaissance subtree."""

    return py_trees.composites.Sequence(
        name="TargetReconnaissance",
        memory=True,
        children=[
            WaitForRCSwitch(
                name="WaitForReconComplete",
                channel=RECON_COMPLETE_RC_CHANNEL,
            ),
            TriggerPostProcessing(),
        ],
    )
