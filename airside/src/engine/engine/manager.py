"""
Entry-point for the airside behavior-tree engine.
"""

from __future__ import annotations

import sys

import py_trees
import py_trees_ros
import rclpy
import rclpy.node
from engine import blackboard_keys
from airside.src.engine.engine.subtrees.lapping import create_lapping_subtree

TICK_PERIOD_MS: float = 500.0  # Tree clock speed
UNICODE_TREE_DEBUG: bool = True  # Whether or not to print the tree with Unicode characters on every tick

LAPPING_DURATION_SEC: float = 600.0  # Lapping deadline, from engine startup


def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Root", memory=True)
    root.add_child(create_lapping_subtree())
    return root


def set_lapping_deadline(node: rclpy.node.Node) -> None:
    """
    Write the lapping deadline to the blackboard for the time checks.
    """

    blackboard = py_trees.blackboard.Client(name="MissionConfig")
    blackboard.register_key(
        key=blackboard_keys.LAPPING_END_TIME_SEC, access=py_trees.common.Access.WRITE
    )
    now_s = node.get_clock().now().nanoseconds / 1e9
    blackboard.set(blackboard_keys.LAPPING_END_TIME_SEC, now_s + LAPPING_DURATION_SEC)


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

    set_lapping_deadline(tree.node)

    tree.tick_tock(period_ms=TICK_PERIOD_MS)

    try:
        if tree.node is not None:
            rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):  #type: ignore[attr-defined]
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
