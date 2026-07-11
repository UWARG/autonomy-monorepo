"""
Bookkeeping behaviors that maintain the lap-timing blackboard entries.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys


class RecordLapStart(py_trees.behaviour.Behaviour):
    """
    Write the current ROS clock time to ``latest_lap_start_time_sec``
    and rewinds ``waypoint_index`` to 0.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "RecordLapStart") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LATEST_LAP_START_TIME_SEC,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        now_s = self._node.get_clock().now().nanoseconds / 1e9
        self.blackboard.set(blackboard_keys.LATEST_LAP_START_TIME_SEC, now_s)
        self.blackboard.set(blackboard_keys.WAYPOINT_INDEX, 0)
        self._node.get_logger().info(f"{self.name}: lap started at t={now_s:.1f}s")

        return py_trees.common.Status.SUCCESS


class RecordLapEnd(py_trees.behaviour.Behaviour):
    """
    Checks if the lap was completed and updates ``estimated_lap_time_sec``
    from the lap that just finished.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "RecordLapEnd") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LATEST_LAP_START_TIME_SEC,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINTS, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.ESTIMATED_LAP_TIME_SEC,
            access=py_trees.common.Access.WRITE,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            lap_start_s = self.blackboard.get(blackboard_keys.LATEST_LAP_START_TIME_SEC)
            waypoints = self.blackboard.get(blackboard_keys.WAYPOINTS)
            index = self.blackboard.get(blackboard_keys.WAYPOINT_INDEX)
        except KeyError:
            self._node.get_logger().warning(
                f"{self.name}: lap bookkeeping keys missing, skipping estimate"
            )
            return py_trees.common.Status.SUCCESS

        lap_time_s = self._node.get_clock().now().nanoseconds / 1e9 - lap_start_s

        if index < len(waypoints):
            self._node.get_logger().info(
                f"{self.name}: lap aborted at waypoint {index}/{len(waypoints)} "
                f"after {lap_time_s:.1f}s, keeping previous lap-time estimate"
            )
            return py_trees.common.Status.SUCCESS

        self.blackboard.set(blackboard_keys.ESTIMATED_LAP_TIME_SEC, lap_time_s)
        self._node.get_logger().info(
            f"{self.name}: lap completed in {lap_time_s:.1f}s"
        )
        return py_trees.common.Status.SUCCESS
