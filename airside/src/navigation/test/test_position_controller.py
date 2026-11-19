"""Decision tests for the position controller."""

import math

import pytest
import rclpy
from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.msg import PositionTarget, TrafficSnapshot
from mavros_msgs.msg import State
from obstacle_avoidance import TrafficAircraft
from sensor_msgs.msg import NavSatFix

from navigation.position_controller import PositionController
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate

NOW = 100.0
HERE = Coordinate(43.0, -80.0, 10.0)
TARGET = enu_to_coordinate(HERE, 0.0, 30.0, 10.0)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def controller():
    node = PositionController()
    node._state = State(connected=True, armed=True, mode="GUIDED")
    node._target, node._target_received_s = TARGET, NOW
    place(node, HERE)
    node._traffic = TrafficSnapshot(connected=True, healthy=True)
    node._traffic_received_s = NOW
    yield node
    node.destroy_node()


def place(
    node: PositionController, position: Coordinate, received_s: float = NOW
) -> None:
    node._fix = NavSatFix(latitude=position.lat, longitude=position.lon)
    node._rel_alt_m = position.alt
    node._fix_received_s = node._rel_alt_received_s = received_s


def test_clear_path_goes_to_lookahead_point_toward_target(controller) -> None:
    setpoint = controller._next_setpoint(NOW)

    assert HERE.lat < setpoint.lat < TARGET.lat
    assert setpoint.lon == pytest.approx(HERE.lon)
    assert setpoint.alt == TARGET.alt


def test_near_target_goes_to_target_itself(controller) -> None:
    controller._target = enu_to_coordinate(HERE, 0.0, 0.5, 10.0)

    assert controller._next_setpoint(NOW) == controller._target


def test_path_aims_lookahead_along_it(controller) -> None:
    previous = enu_to_coordinate(TARGET, 0.0, -30.0, 10.0)
    controller._path = [previous, TARGET, enu_to_coordinate(TARGET, 30.0, 0.0, 10.0)]
    controller._target_index = 1
    place(controller, enu_to_coordinate(TARGET, 0.0, -2.0, 10.0))
    setpoint = controller._next_setpoint(NOW)

    # Past the target and bending onto the next leg rather than stopping.
    assert setpoint.lat > TARGET.lat
    assert setpoint.lon > TARGET.lon


def test_spline_is_reused_until_the_path_changes(controller) -> None:
    previous = enu_to_coordinate(TARGET, 0.0, -30.0, 10.0)
    controller._path = [previous, TARGET, enu_to_coordinate(TARGET, 30.0, 0.0, 10.0)]
    controller._target_index = 1
    controller._next_setpoint(NOW)
    spline = controller._spline
    controller._next_setpoint(NOW)

    assert controller._spline is spline
    controller._path = [previous, TARGET]
    controller._next_setpoint(NOW)
    assert controller._spline is not spline


def test_path_ending_at_target_stops_there(controller) -> None:
    controller._path = [enu_to_coordinate(TARGET, 0.0, -30.0, 10.0), TARGET]
    controller._target_index = 1
    place(controller, enu_to_coordinate(TARGET, 0.0, -0.5, 10.0))

    assert controller._next_setpoint(NOW) == TARGET


def test_path_not_through_target_is_ignored(controller) -> None:
    msg = PositionTarget(
        target=CoordinateMsg(lat=TARGET.lat, lon=TARGET.lon, alt=TARGET.alt),
        path=[CoordinateMsg(lat=HERE.lat, lon=HERE.lon, alt=HERE.alt)],
        target_index=0,
    )
    controller._target_callback(msg)

    assert controller._path == []


def test_target_message_without_path_clears_it(controller) -> None:
    controller._path = [HERE, TARGET]
    controller._target_callback(
        PositionTarget(
            target=CoordinateMsg(lat=TARGET.lat, lon=TARGET.lon, alt=TARGET.alt)
        )
    )

    assert controller._path == []


def test_traffic_on_the_line_bends_the_setpoint(controller) -> None:
    blocker = enu_to_coordinate(HERE, 0.0, 12.0)
    controller._traffic_aircraft = (
        TrafficAircraft(1, blocker.lat, blocker.lon, 10.0, 0.0, 0.0, 2.0, 5.0),
    )

    assert controller._next_setpoint(NOW).lon != pytest.approx(HERE.lon, abs=1e-6)


def test_persistent_blockage_holds_and_alerts_pilot(controller) -> None:
    assert controller._hold(HERE, "BLOCKED", NOW) == HERE
    assert controller._last_alert_s == -math.inf
    assert controller._hold(HERE, "BLOCKED", NOW + 5.0) == HERE
    assert controller._last_alert_s == NOW + 5.0


def test_predicted_hold_conflict_alerts_immediately(controller) -> None:
    assert controller._hold(HERE, "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED", NOW) == HERE
    assert controller._last_alert_s == NOW


def test_crossing_yield_alerts_only_if_it_persists(controller) -> None:
    assert controller._hold(HERE, "YIELD_CROSSING", NOW) == HERE
    assert controller._last_alert_s == -math.inf
    assert controller._hold(HERE, "YIELD_CROSSING", NOW + 19.9) == HERE
    assert controller._last_alert_s == -math.inf
    assert controller._hold(HERE, "YIELD_CROSSING", NOW + 20.0) == HERE
    assert controller._last_alert_s == NOW + 20.0


def test_unconfirmed_own_index_holds(controller) -> None:
    controller._traffic.own_aircraft_index = -1
    assert controller._next_setpoint(NOW) == HERE


def test_own_aircraft_in_snapshot_holds(controller) -> None:
    controller._traffic.own_aircraft_index = 2
    controller._traffic_aircraft = (
        TrafficAircraft(2, HERE.lat, HERE.lon, 10.0, 0.0, 0.0, 2.0, 5.0),
    )
    assert controller._next_setpoint(NOW) == HERE


@pytest.mark.parametrize(
    "unusable",
    [
        lambda c: setattr(c, "_traffic", None),
        lambda c: setattr(c, "_traffic", TrafficSnapshot(connected=False)),
        lambda c: setattr(
            c, "_traffic", TrafficSnapshot(connected=True, healthy=False)
        ),
        lambda c: setattr(c, "_traffic_received_s", NOW - 5.0),
        lambda c: setattr(c, "_fix_received_s", NOW - 2.0),
    ],
)
def test_unusable_inputs_hold_where_the_hold_began(controller, unusable) -> None:
    unusable(controller)
    first = controller._next_setpoint(NOW)
    place(
        controller, enu_to_coordinate(HERE, 3.0, 3.0, 10.0), controller._fix_received_s
    )
    later = controller._next_setpoint(NOW)

    assert first == HERE
    assert later == HERE


def test_pilot_control_publishes_nothing(controller) -> None:
    controller._state = State(connected=True, armed=True, mode="LOITER")

    assert controller._next_setpoint(NOW) is None


def test_target_timeout_holds_once_then_lets_go(controller) -> None:
    controller._next_setpoint(NOW)
    place(controller, HERE, NOW + 2.0)

    assert controller._next_setpoint(NOW + 2.0) == HERE
    assert controller._next_setpoint(NOW + 2.1) is None


def test_no_target_ever_publishes_nothing(controller) -> None:
    controller._target, controller._target_received_s = None, -math.inf

    assert controller._next_setpoint(NOW) is None
