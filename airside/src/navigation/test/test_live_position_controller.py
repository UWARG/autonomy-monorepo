from __future__ import annotations

import time

import pytest
import rclpy
from airside_interfaces.msg import Coordinate, ObstacleSnapshotStatus
from mavros_msgs.msg import GlobalPositionTarget, State
from navigation.live_position_controller_node import PositionController
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


@pytest.fixture
def controller() -> PositionController:
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "traffic_required:=true",
            "-p",
            "own_aircraft_index:=1",
        ]
    )
    node = PositionController()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _ready_navigation(controller: PositionController) -> None:
    state = State(connected=True, armed=True, mode="GUIDED")
    fix = NavSatFix(latitude=43.47, longitude=-80.54)
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.header.stamp = controller.get_clock().now().to_msg()
    controller._state_callback(state)
    controller._fix_callback(fix)
    controller._altitude_callback(Float64(data=15.0))


def _fresh_empty_snapshot(controller: PositionController, sequence: int = 1) -> None:
    controller._snapshot_callback(
        ObstacleSnapshotStatus(
            sequence=sequence,
            connected=True,
            healthy=True,
            aircraft_indices=[],
        )
    )


def _target(controller: PositionController) -> None:
    controller._target_callback(Coordinate(lat=43.4701, lon=-80.54, alt=15.0))


def test_complete_snapshot_allows_setpoint(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    _fresh_empty_snapshot(controller)
    _target(controller)

    controller._control_cycle()

    assert len(published) == 1
    assert published[0].lat == pytest.approx(43.4701)


def test_disconnected_traffic_holds_current_position(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    _target(controller)
    controller._snapshot_callback(
        ObstacleSnapshotStatus(
            sequence=1,
            connected=False,
            healthy=False,
            reason="AEAC_DISCONNECTED",
        )
    )

    controller._control_cycle()

    assert len(published) == 1
    assert published[0].lat == pytest.approx(43.47)
    assert published[0].lon == pytest.approx(-80.54)
    assert published[0].alt == pytest.approx(15.0)


def test_stale_traffic_holds_current_position(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    _fresh_empty_snapshot(controller)
    _target(controller)
    assert controller._traffic_store is not None
    controller._traffic_store._latest_received_s = time.monotonic() - 3.0

    controller._control_cycle()

    assert len(published) == 1
    assert published[0].lat == pytest.approx(43.47)


def test_missing_position_never_publishes_hold(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    controller._state_callback(State(connected=True, armed=True, mode="GUIDED"))
    _fresh_empty_snapshot(controller)
    _target(controller)

    controller._control_cycle()

    assert published == []


def test_pilot_takeover_clears_target_and_stops_setpoints(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[GlobalPositionTarget] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    _fresh_empty_snapshot(controller)
    _target(controller)

    controller._state_callback(State(connected=True, armed=True, mode="LOITER"))
    controller._control_cycle()

    assert controller._target is None
    assert published == []


def test_target_received_during_pilot_control_is_not_replayed_on_guided_resume(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    _fresh_empty_snapshot(controller)
    controller._state_callback(State(connected=True, armed=True, mode="LOITER"))
    _target(controller)

    controller._state_callback(State(connected=True, armed=True, mode="GUIDED"))
    controller._control_cycle()

    assert controller._target is None
    assert published == []


@pytest.mark.parametrize(
    ("latitude", "longitude"),
    [(91.0, -80.54), (43.47, -181.0)],
)
def test_out_of_range_gps_does_not_publish_setpoint(
    controller: PositionController,
    monkeypatch: pytest.MonkeyPatch,
    latitude: float,
    longitude: float,
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    assert controller._latest_fix is not None
    controller._latest_fix.latitude = latitude
    controller._latest_fix.longitude = longitude
    _fresh_empty_snapshot(controller)
    _target(controller)

    controller._control_cycle()

    assert published == []


def test_no_route_is_a_not_ready_hold(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    diagnostics: list[tuple[str, bool]] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    monkeypatch.setattr(
        controller,
        "_publish_diagnostic",
        lambda reason, *, ready: diagnostics.append((reason, ready)),
    )
    monkeypatch.setattr(
        controller,
        "_planned_target",
        lambda target, obstacles: (controller._hold_target(), "hold"),
    )
    _ready_navigation(controller)
    _fresh_empty_snapshot(controller)
    _target(controller)

    controller._control_cycle()

    assert published[0].lat == pytest.approx(43.47)
    assert diagnostics[-1] == ("hold", False)


def test_invalid_target_replaces_progress_with_hold(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    _fresh_empty_snapshot(controller)
    _target(controller)
    controller._target_callback(Coordinate(lat=91.0, lon=-80.54, alt=15.0))

    controller._control_cycle()

    assert controller._target is None
    assert published[0].lat == pytest.approx(43.47)
    assert published[0].alt == pytest.approx(15.0)


def test_republished_old_gps_fix_does_not_publish_setpoint(
    controller: PositionController, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[Coordinate] = []
    monkeypatch.setattr(controller, "_publish_setpoint", published.append)
    _ready_navigation(controller)
    assert controller._latest_fix is not None
    controller._latest_fix.header.stamp.sec -= 5
    _fresh_empty_snapshot(controller)
    _target(controller)

    controller._control_cycle()

    assert published == []
