"""
Bookkeeping behaviors that maintain the lap-timing blackboard entries.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import LAPPING_DURATION_SEC
from engine.ground_log import send_to_ground


class SetLappingDeadline(py_trees.behaviour.Behaviour):
    """
    Writes the lapping deadline to the blackboard when lapping begins.

        Always returns SUCCESS.
    """

    def __init__(self, name: str = "SetLappingDeadline") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAPPING_END_TIME_SEC,
            access=py_trees.common.Access.WRITE,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        now_s = self._node.get_clock().now().nanoseconds / 1e9
        self.blackboard.set(
            blackboard_keys.LAPPING_END_TIME_SEC, now_s + LAPPING_DURATION_SEC
        )
        self._node.get_logger().info(
            f"{self.name}: lapping deadline set to t={now_s + LAPPING_DURATION_SEC:.1f}s"
        )
        send_to_ground(
            self._node, f"ENG: lapping phase, {LAPPING_DURATION_SEC:.0f}s window"
        )
        return py_trees.common.Status.SUCCESS


class ResetLapWaypoints(py_trees.behaviour.Behaviour):
    """
    Rewinds ``waypoint_index`` to 0 at the start of a lap.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "ResetLapWaypoints") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        self.blackboard.set(blackboard_keys.WAYPOINT_INDEX, 0)
        return py_trees.common.Status.SUCCESS


class StartLapTimer(py_trees.behaviour.Behaviour):
    """
    Starts the lap timer once the drone reaches the lap's first waypoint.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "StartLapTimer") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.LATEST_LAP_START_TIME_SEC,
            access=py_trees.common.Access.WRITE,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            index = self.blackboard.get(blackboard_keys.WAYPOINT_INDEX)
        except KeyError:
            self._node.get_logger().warning(
                f"{self.name}: no waypoint index on the blackboard"
            )
            return py_trees.common.Status.SUCCESS

        if index != 1:
            return py_trees.common.Status.SUCCESS

        now_s = self._node.get_clock().now().nanoseconds / 1e9
        self.blackboard.set(blackboard_keys.LATEST_LAP_START_TIME_SEC, now_s)
        self._node.get_logger().info(
            f"{self.name}: first waypoint reached, lap started at t={now_s:.1f}s"
        )
        send_to_ground(self._node, "ENG: lap started")
        return py_trees.common.Status.SUCCESS


class RecordLapEnd(py_trees.behaviour.Behaviour):
    """
    Checks if the lap was completed and updates ``estimated_lap_time_sec``
    from the lap that just finished.

    The first completed lap is not used as an estimate.

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
        self._completed_laps = 0

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
            send_to_ground(
                self._node, f"ENG: lap aborted at wp {index}/{len(waypoints)}"
            )
            return py_trees.common.Status.SUCCESS

        self._completed_laps += 1
        send_to_ground(
            self._node,
            f"ENG: lap {self._completed_laps} done in {lap_time_s:.0f}s",
        )
        if self._completed_laps == 1:
            self._node.get_logger().info(
                f"{self.name}: first lap completed in {lap_time_s:.1f}s"
            )
            return py_trees.common.Status.SUCCESS

        self.blackboard.set(blackboard_keys.ESTIMATED_LAP_TIME_SEC, lap_time_s)
        self._node.get_logger().info(
            f"{self.name}: lap completed in {lap_time_s:.1f}s"
        )
        return py_trees.common.Status.SUCCESS
