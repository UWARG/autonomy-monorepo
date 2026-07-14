"""
Entry-point for the airside behavior-tree engine.
"""

from __future__ import annotations

import sys

import py_trees
import py_trees_ros
import rclpy
from engine.behaviors.comms.configure_stream_rates import ConfigureStreamRates
from engine.behaviors.navigation.load_waypoint_list import LoadWaypointList
from engine.behaviors.navigation.takeoff import Takeoff
from engine.behaviors.rc.rc_switch import KillSwitch
from engine.constants import (
    RC_SWITCHES_ENABLED,
    TICK_PERIOD_MS,
    UNICODE_TREE_DEBUG,
)
from engine.subtrees.land import create_land_subtree
from engine.subtrees.lapping import create_lapping_subtree
from engine.subtrees.target_reconnaissance import (
    create_target_reconnaissance_subtree,
)


def create_root() -> py_trees.behaviour.Behaviour:
    """Build the full-auto mission tree."""
    mission = py_trees.composites.Sequence(
        name="Mission",
        memory=True,
        children=[
            ConfigureStreamRates(),
            LoadWaypointList(),
            Takeoff(),
            create_lapping_subtree(),
            create_target_reconnaissance_subtree(),
            create_land_subtree(),
            py_trees.behaviours.Running(name="MissionComplete"),
        ],
    )

    if RC_SWITCHES_ENABLED:
        return KillSwitch(child=mission)
    return mission


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=UNICODE_TREE_DEBUG,
    )

    try:
        tree.setup(node_name="engine_manager", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError:
        if tree.node is not None:
            tree.node.get_logger().error(
                "Failed to set up the behavior tree within the timeout."
            )
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        rclpy.try_shutdown()
        return

    tree.tick_tock(period_ms=TICK_PERIOD_MS)

    try:
        if tree.node is not None:
            rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):  # type: ignore[attr-defined]
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
