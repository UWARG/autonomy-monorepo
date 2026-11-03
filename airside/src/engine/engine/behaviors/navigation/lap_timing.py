"""
Bookkeeping behaviors that track and report lap progress and lap times.
"""

from __future__ import annotations

import py_trees
import rclpy.node

from engine import blackboard_keys
from engine.ground_log import send_to_ground


class ResetLapWaypoints(py_trees.behaviour.Behaviour):
    """
    Rewinds ``waypoint_index`` at the start of a lap: to 1 if the last lap
    closed (the drone is already at the first waypoint). Clears ``lap_closed``.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "ResetLapWaypoints") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=blackboard_keys.LAP_CLOSED, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            closed = self.blackboard.get(blackboard_keys.LAP_CLOSED)
        except KeyError:
            closed = False
        self.blackboard.set(blackboard_keys.WAYPOINT_INDEX, 1 if closed else 0)
        self.blackboard.set(blackboard_keys.LAP_CLOSED, False)
        return py_trees.common.Status.SUCCESS


class RecordWaypointReached(py_trees.behaviour.Behaviour):
    """
    Lap bookkeeping once the drone reaches a lap waypoint: starts the lap
    timer on reaching the first waypoint from elsewhere, and marks
    ``lap_closed`` on returning to it at the end of the lap.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "RecordWaypointReached") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINTS, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.LATEST_LAP_START_TIME_SEC,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key=blackboard_keys.LAP_CLOSED, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            waypoints = self.blackboard.get(blackboard_keys.WAYPOINTS)
            index = self.blackboard.get(blackboard_keys.WAYPOINT_INDEX)
        except KeyError:
            self._node.get_logger().warning(
                f"{self.name}: no waypoint list or index on the blackboard"
            )
            return py_trees.common.Status.SUCCESS

        # LoadNextWaypoint advances the index past the waypoint it loads.
        if index == len(waypoints) + 1:
            self.blackboard.set(blackboard_keys.LAP_CLOSED, True)
        elif index == 1:
            now_s = self._node.get_clock().now().nanoseconds / 1e9
            self.blackboard.set(blackboard_keys.LATEST_LAP_START_TIME_SEC, now_s)
            self._node.get_logger().info(
                f"{self.name}: first waypoint reached, lap started at t={now_s:.1f}s"
            )
            send_to_ground(self._node, "ENG: lap started")
        return py_trees.common.Status.SUCCESS


class RecordLapEnd(py_trees.behaviour.Behaviour):
    """
    Reports whether the lap that just finished was completed (back at the
    first waypoint) or cut short, and how long it took.

    Always returns SUCCESS.
    """

    def __init__(self, name: str = "RecordLapEnd") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LATEST_LAP_START_TIME_SEC,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINTS, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINT_INDEX, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=blackboard_keys.LAP_CLOSED, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._completed_laps = 0

    def update(self) -> py_trees.common.Status:
        try:
            lap_start_s = self.blackboard.get(blackboard_keys.LATEST_LAP_START_TIME_SEC)
            waypoints = self.blackboard.get(blackboard_keys.WAYPOINTS)
            index = self.blackboard.get(blackboard_keys.WAYPOINT_INDEX)
            closed = self.blackboard.get(blackboard_keys.LAP_CLOSED)
        except KeyError:
            self._node.get_logger().warning(
                f"{self.name}: lap bookkeeping keys missing, skipping lap report"
            )
            return py_trees.common.Status.SUCCESS

        now_s = self._node.get_clock().now().nanoseconds / 1e9
        lap_time_s = now_s - lap_start_s

        if not closed:
            # The waypoint being flown to when the lap was cut short.
            waypoint = (index - 1) % len(waypoints) + 1
            self._node.get_logger().info(
                f"{self.name}: lap aborted at waypoint {waypoint}/{len(waypoints)} "
                f"after {lap_time_s:.1f}s"
            )
            send_to_ground(
                self._node, f"ENG: lap aborted at wp {waypoint}/{len(waypoints)}"
            )
            return py_trees.common.Status.SUCCESS

        self._completed_laps += 1
        self.blackboard.set(blackboard_keys.LATEST_LAP_START_TIME_SEC, now_s)
        send_to_ground(
            self._node,
            f"ENG: lap {self._completed_laps} done in {lap_time_s:.0f}s",
        )
        self._node.get_logger().info(
            f"{self.name}: lap {self._completed_laps} completed in {lap_time_s:.1f}s"
        )
        return py_trees.common.Status.SUCCESS
