from __future__ import annotations

from types import SimpleNamespace

import py_trees

from airside_interfaces.msg import PositionTarget
from engine import blackboard_keys
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.behaviors.navigation.lap_timing import RecordLapEnd
from engine.constants import GUIDED_MODE
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix


class _Logger:
    def info(self, *_args, **_kwargs) -> None:
        pass

    def warning(self, *_args, **_kwargs) -> None:
        pass

    def error(self, *_args, **_kwargs) -> None:
        pass


class _Publisher:
    def publish(self, _message) -> None:
        pass


def test_waypoint_navigation_does_not_fail_after_a_deadline() -> None:
    behavior = FlyToWaypoint()
    behavior._node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=121_000_000_000)
        ),
        get_logger=lambda: _Logger(),
    )
    behavior._target_pub = _Publisher()
    behavior._target = PositionTarget()
    behavior._target.target.lat = 0.0
    behavior._target.target.lon = 0.001
    behavior._target.target.alt = 10.0
    setattr(behavior, "_start_time_s", 0.0)
    behavior._latest_state = State()
    behavior._latest_state.mode = GUIDED_MODE
    behavior._latest_fix = NavSatFix()
    behavior._latest_rel_alt_m = 10.0

    assert behavior.update() is py_trees.common.Status.RUNNING


def test_abeam_pass_requires_waypoint_altitude() -> None:
    behavior = FlyToWaypoint(acceptance_radius_m=3.0, fly_through=True)
    behavior._target = PositionTarget()
    behavior._target.target.lat = 0.0
    behavior._target.target.lon = 0.001
    behavior._leg_start = NavSatFix()

    assert behavior._passed_abeam(east_m=-1.0, north_m=0.0, up_m=0.0)
    assert not behavior._passed_abeam(east_m=-1.0, north_m=0.0, up_m=4.0)


def test_incomplete_lap_fails_instead_of_starting_another(monkeypatch) -> None:
    behavior = RecordLapEnd()
    behavior.setup(node=_FakeNode())
    writer = py_trees.blackboard.Client(name="test_incomplete_lap_writer")
    values = {
        blackboard_keys.LATEST_LAP_START_TIME_SEC: 1.0,
        blackboard_keys.WAYPOINTS: [object(), object()],
        blackboard_keys.WAYPOINT_INDEX: 2,
        blackboard_keys.LAP_CLOSED: False,
    }
    for key, value in values.items():
        writer.register_key(key=key, access=py_trees.common.Access.WRITE)
        writer.set(key=key, value=value)

    monkeypatch.setattr(
        "engine.behaviors.navigation.lap_timing.send_to_ground",
        lambda *_args, **_kwargs: None,
    )

    assert behavior.update() is py_trees.common.Status.FAILURE


class _FakeNode:
    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)
        )

    def get_logger(self):
        return _Logger()
