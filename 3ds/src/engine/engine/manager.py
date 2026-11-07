"""
Entry-point for the 3DS behavior-tree engine.
"""

from __future__ import annotations

import sys

import py_trees
import py_trees_ros
import rclpy
from engine.behaviors.comms.configure_stream_rates import ConfigureStreamRates
from engine.behaviors.guided_mode_gate import PauseUnlessGuided
from engine.behaviors.navigation.record_launch_point import RecordLaunchPoint
from engine.behaviors.navigation.takeoff import Takeoff
from engine.constants import TICK_PERIOD_MS, UNICODE_TREE_DEBUG
from engine.subtrees.land import create_land_subtree
from engine.subtrees.setpoints import create_setpoints_subtree


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Builds the full mission tree: photograph each setpoint sent from the
    ground until the go-home command, then return to and land at the launch
    point.

    RecordLaunchPoint waits for the pilot to arm and records where the drone
    took off.

    The engine never changes flight mode. PauseUnlessGuided freezes the
    mission (without resetting its progress) until the pilot selects GUIDED,
    and again whenever the pilot takes back control, so returning to GUIDED
    resumes the mission where it left off. The final MissionComplete node
    holds the tree in RUNNING after landing.

    ```
    Root [Sequence]
    ├── ConfigureStreamRates
    └── PauseUnlessGuided
        └── Mission [Sequence]
            ├── RecordLaunchPoint
            ├── Takeoff
            ├── UntilGoHome (setpoints subtree)
            ├── LandPhase
            └── MissionComplete [Running]
    ```
    """

    mission = py_trees.composites.Sequence(
        name="Mission",
        memory=True,
        children=[
            RecordLaunchPoint(),
            Takeoff(),
            create_setpoints_subtree(),
            create_land_subtree(),
            py_trees.behaviours.Running(name="MissionComplete"),
        ],
    )

    return py_trees.composites.Sequence(
        name="Root",
        memory=True,
        children=[
            ConfigureStreamRates(),
            PauseUnlessGuided(child=mission),
        ],
    )


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
