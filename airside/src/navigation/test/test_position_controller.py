"""Decision tests for the position controller."""

import math

import pytest
import rclpy
from airside_interfaces.msg import TrafficSnapshot
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


def place(node: PositionController, position: Coordinate, received_s: float = NOW) -> None:
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


def test_traffic_on_the_line_bends_the_setpoint(controller) -> None:
    blocker = enu_to_coordinate(HERE, 0.0, 6.0)
    controller._traffic_aircraft = (
        TrafficAircraft(1, blocker.lat, blocker.lon, 10.0, 0.0, 0.0, 2.0, 5.0),
    )

    assert controller._next_setpoint(NOW).lon != pytest.approx(HERE.lon, abs=1e-6)


@pytest.mark.parametrize(
    "unusable",
    [
        lambda c: setattr(c, "_traffic", None),
        lambda c: setattr(c, "_traffic", TrafficSnapshot(connected=False)),
        lambda c: setattr(c, "_traffic", TrafficSnapshot(connected=True, healthy=False)),
        lambda c: setattr(c, "_traffic_received_s", NOW - 5.0),
        lambda c: setattr(c, "_fix_received_s", NOW - 2.0),
    ],
)
def test_unusable_inputs_hold_where_the_hold_began(controller, unusable) -> None:
    unusable(controller)
    first = controller._next_setpoint(NOW)
    place(controller, enu_to_coordinate(HERE, 3.0, 3.0, 10.0), controller._fix_received_s)
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
