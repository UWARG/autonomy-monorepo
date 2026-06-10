"""
Entry-point for the airside behavior-tree engine.
"""

from __future__ import annotations

import sys

import py_trees
import py_trees_ros
import rclpy
from engine.behaviors.read_camera import ReadCameraBehavior

TICK_PERIOD_MS: float = 500.0 # Tree clock speed
UNICODE_TREE_DEBUG: bool = True # Whether or not to print the tree with Unicode characters on every tick


def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Root", memory=False)
    root.add_child(ReadCameraBehavior(name="ReadCamera"))
    return root

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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): #type: ignore
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
