"""A stalled lap must not silently become a completed lap."""

from unittest.mock import Mock

import py_trees
from airside_interfaces.msg import Coordinate, PositionTarget
from engine import blackboard_keys
from engine.behaviors.navigation import fly_to_waypoint
from engine.behaviors.navigation import lap_timing
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.behaviors.navigation.lap_timing import RecordLapEnd
from engine.constants import WAYPOINT_NAV_TIMEOUT_S
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix


def test_delayed_waypoint_keeps_target_and_alerts_once(monkeypatch) -> None:
    send_to_ground = Mock()
    monkeypatch.setattr(fly_to_waypoint, "send_to_ground", send_to_ground)
    behavior = FlyToWaypoint()
    behavior._node = Mock()
    behavior._target_pub = Mock()
    behavior._target = PositionTarget(
        target=Coordinate(lat=43.001, lon=-80.0, alt=10.0)
    )
    behavior._latest_state = State(mode="GUIDED")
    behavior._latest_fix = NavSatFix(latitude=43.0, longitude=-80.0)
    behavior._latest_rel_alt_m = 10.0
    behavior._start_time_s = 0.0
    behavior._timeout_alerted = False
    monkeypatch.setattr(behavior, "_now_s", lambda: WAYPOINT_NAV_TIMEOUT_S + 1.0)

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior._target is not None
    assert behavior._target_pub.publish.call_count == 2
    send_to_ground.assert_called_once_with(
        behavior._node, "ENG: waypoint delayed, pilot take over"
    )


def test_incomplete_lap_never_advances_to_next_mission_phase(monkeypatch) -> None:
    send_to_ground = Mock()
    monkeypatch.setattr(lap_timing, "send_to_ground", send_to_ground)
    behavior = RecordLapEnd()
    behavior.setup(node=Mock())
    values = {
        blackboard_keys.LATEST_LAP_START_TIME_SEC: 0.0,
        blackboard_keys.WAYPOINTS: [1, 2, 3],
        blackboard_keys.WAYPOINT_INDEX: 2,
        blackboard_keys.LAP_CLOSED: False,
    }
    behavior.blackboard = Mock()
    behavior.blackboard.get.side_effect = values.__getitem__
    behavior._node.get_clock.return_value.now.return_value.nanoseconds = 200_000_000_000

    assert behavior.update() == py_trees.common.Status.RUNNING
    assert behavior.update() == py_trees.common.Status.RUNNING
    send_to_ground.assert_called_once_with(
        behavior._node, "ENG: lap blocked at wp 2, pilot take over"
    )
