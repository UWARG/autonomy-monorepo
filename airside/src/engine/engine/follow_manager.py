"""Dedicated target-follow behavior tree and high-rate streamer entry point."""

from __future__ import annotations

import sys

import py_trees
import py_trees_ros
import rclpy
from stack_config import DEPLOYED

from engine.behaviors.emergency_stop import EmergencyStopBehavior
from engine.behaviors.follow import FollowBehavior
from engine.behaviors.read_target import (
    KEY_COMMAND,
    KEY_COMMAND_STAMP,
    KEY_ESTOP_EMERGENCY,
    KEY_ESTOP_HOLD,
    KEY_ESTOP_RECEDE,
    KEY_RANGE_M,
    KEY_TARGET_M,
    ReadTargetBehavior,
)
from engine.setpoint_streamer import SetpointStreamer


def create_root() -> py_trees.behaviour.Behaviour:
    """Build the 2 Hz high-level arbitration tree.

    The streamer independently recomputes raw safety and filtered control at
    high rate. This tree remains the inspectable mission-level policy.
    """
    arbiter = py_trees.composites.Selector(name="EmergencyOrFollow", memory=False)
    arbiter.add_children(
        [
            EmergencyStopBehavior(
                config=DEPLOYED.safety,
                recede_speed=DEPLOYED.recede_speed,
            ),
            FollowBehavior(config=DEPLOYED.follow),
        ]
    )
    return py_trees.composites.Sequence(
        name="FollowMission",
        memory=False,
        children=[ReadTargetBehavior(), arbiter],
    )


def _initialize_blackboard() -> None:
    client = py_trees.blackboard.Client(name="follow_init")
    defaults = {
        KEY_TARGET_M: None,
        KEY_RANGE_M: None,
        KEY_COMMAND: None,
        KEY_COMMAND_STAMP: 0.0,
        KEY_ESTOP_EMERGENCY: False,
        KEY_ESTOP_RECEDE: False,
        KEY_ESTOP_HOLD: False,
    }
    for key, value in defaults.items():
        client.register_key(key=key, access=py_trees.common.Access.WRITE)
        client.set(key, value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    _initialize_blackboard()
    tree = py_trees_ros.trees.BehaviourTree(
        root=create_root(),
        unicode_tree_debug=False,
    )
    try:
        tree.setup(node_name="follow_manager", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError:
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        rclpy.try_shutdown()
        return

    if tree.node is None:
        rclpy.try_shutdown()
        return
    SetpointStreamer(tree.node, config=DEPLOYED)
    tree.tick_tock(period_ms=DEPLOYED.tree_period_ms)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):  # type: ignore[attr-defined]
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
