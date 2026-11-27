"""
Publishes a summary of the behaviour tree for the ground station.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from airside_interfaces.msg import MissionStatus
from engine import blackboard_keys
from engine.behaviors.guided_mode_gate import PauseUnlessGuided
from engine.constants import MISSION_STATUS_PERIOD_S
from engine.ground_log import last_message
from rclpy.qos import DurabilityPolicy, QoSProfile

_STATUS_TOPIC = "engine/status"
_LAP_COUNT_PARAMETER = "lap_count"

# Node names from manager.create_root and the subtrees.
_MISSION = "Mission"
_ABORT = "Abort"
_LAPPING = "Lapping"
_MISSION_COMPLETE = "MissionComplete"


class MissionStatusPublisher:
    """
    Summarizes the tree on ``engine/status`` after a tick, at most every
    ``MISSION_STATUS_PERIOD_S``: the mission step and behaviour running,
    lap and waypoint progress, and the latest ground message.
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        self._node = node
        self._last_publish_ns: int | None = None
        self._publisher = node.create_publisher(
            MissionStatus,
            _STATUS_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._blackboard = py_trees.blackboard.Client(name="MissionStatus")
        for key in (
            blackboard_keys.LAPS_REMAINING,
            blackboard_keys.WAYPOINTS,
            blackboard_keys.WAYPOINT_INDEX,
        ):
            self._blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def publish(self, tree: py_trees.trees.BehaviourTree) -> None:
        now_ns = self._node.get_clock().now().nanoseconds
        if (
            self._last_publish_ns is not None
            and now_ns - self._last_publish_ns < MISSION_STATUS_PERIOD_S * 1e9
        ):
            return
        self._last_publish_ns = now_ns

        nodes = {node.name: node for node in tree.root.iterate()}
        tip = tree.root.tip()

        status = MissionStatus()
        status.behaviour = tip.name if tip is not None else ""
        status.last_message = last_message()

        abort = nodes.get(_ABORT)
        mission = nodes.get(_MISSION)
        gate = next(
            (node for node in nodes.values() if isinstance(node, PauseUnlessGuided)),
            None,
        )
        mission_started = (
            mission is not None
            and mission.status != py_trees.common.Status.INVALID
            and mission.current_child is not None
        )
        status.phase = mission.current_child.name if mission_started else ""

        if abort is not None and abort.status in (
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.SUCCESS,
        ):
            status.state = "ABORTED"
            status.phase = _ABORT
        elif status.phase == _MISSION_COMPLETE:
            status.state = "COMPLETE"
        elif gate is not None and gate.paused:
            status.state = "PAUSED"
        elif not mission_started:
            status.state = "STARTING"
            status.phase = tip.name if tip is not None else ""
        else:
            status.state = "RUNNING"

        if status.phase == _LAPPING:
            self._fill_lap_progress(status)

        self._publisher.publish(status)

    def _fill_lap_progress(self, status: MissionStatus) -> None:
        if self._node.has_parameter(_LAP_COUNT_PARAMETER):
            status.lap_count = int(self._node.get_parameter(_LAP_COUNT_PARAMETER).value)
        laps_remaining = self._get(blackboard_keys.LAPS_REMAINING)
        if laps_remaining is not None and status.lap_count:
            # AnotherLapRemaining consumes a lap as it starts.
            status.lap = max(0, status.lap_count - laps_remaining)

        waypoints = self._get(blackboard_keys.WAYPOINTS)
        index = self._get(blackboard_keys.WAYPOINT_INDEX)
        if waypoints:
            status.waypoint_count = len(waypoints)
            # LoadNextWaypoint advances the index past the waypoint it loads.
            if index:
                status.waypoint = (index - 1) % len(waypoints) + 1

    def _get(self, key: str):
        try:
            return self._blackboard.get(key)
        except KeyError:
            return None
