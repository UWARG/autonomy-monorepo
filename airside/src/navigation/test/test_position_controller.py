"""Decision tests for the position controller."""

import json
import math
import time
from unittest.mock import Mock

import pytest
import rclpy
from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.msg import (
    PositionTarget,
    TrafficAircraft as TrafficAircraftMsg,
    TrafficSnapshot,
)
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
def controller(monkeypatch):
    corners = [
        enu_to_coordinate(HERE, east, north)
        for east, north in ((-100, -100), (100, -100), (100, 100), (-100, 100))
    ]
    monkeypatch.setenv(
        "FLIGHT_BOUNDARY_WGS84",
        json.dumps([[point.lat, point.lon] for point in corners]),
    )
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


def test_missing_flight_boundary_holds_and_alerts(controller) -> None:
    controller._boundary = None
    controller._boundary_error = "FLIGHT_BOUNDARY_UNCONFIGURED"

    assert controller._next_setpoint(NOW) == HERE
    assert controller._hold_reason == "FLIGHT_BOUNDARY_UNCONFIGURED"
    assert controller._last_alert_s == NOW


def test_target_outside_flight_boundary_holds(controller) -> None:
    controller._target = enu_to_coordinate(HERE, 200.0, 0.0)

    assert controller._next_setpoint(NOW) == HERE
    assert controller._hold_reason == "TARGET_OUTSIDE_FLIGHT_BOUNDARY"


def test_stuck_progress_excludes_old_heading(controller) -> None:
    controller._progress_anchor = HERE
    controller._progress_active_s = controller.STUCK_AFTER_ACTIVE_S
    controller._last_path_s = NOW - 0.1
    controller._last_command_heading_rad = math.pi / 2

    setpoint = controller._next_setpoint(NOW)

    assert len(controller._tried_headings_rad) == 1
    assert setpoint is not None
    assert setpoint.lon != pytest.approx(HERE.lon, abs=1e-7)


def test_lateral_motion_does_not_retry_failed_heading(controller) -> None:
    controller._update_progress(HERE, NOW)
    controller._tried_headings_rad = [math.pi / 2]
    sideways = enu_to_coordinate(HERE, 2.0, 0.0)

    controller._update_progress(sideways, NOW + 1.0)

    assert controller._progress_anchor == sideways
    assert controller._tried_headings_rad == [math.pi / 2]
    toward_target = enu_to_coordinate(HERE, 0.0, 2.0)
    controller._update_progress(toward_target, NOW + 2.0)
    assert controller._tried_headings_rad == []


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


def test_blocked_required_waypoint_holds_without_skipping(controller) -> None:
    controller._traffic_aircraft = (
        TrafficAircraft(2, TARGET.lat, TARGET.lon, 10.0, 0.0, 0.0, 5.0, 5.0),
    )

    assert controller._next_setpoint(NOW) == HERE
    assert controller._hold_reason == "WAYPOINT_IN_TRAFFIC_BUFFER"


def test_blocked_lap_waypoint_proposes_one_outward_replacement(controller) -> None:
    route = [
        enu_to_coordinate(HERE, east, north, 10.0)
        for east, north in ((10, 10), (10, -10), (-10, -10), (-10, 10))
    ]
    original = route[0]
    controller._target_callback(
        PositionTarget(
            target=CoordinateMsg(lat=original.lat, lon=original.lon, alt=10.0),
            path=[CoordinateMsg(lat=original.lat, lon=original.lon, alt=10.0)],
            target_index=0,
            leg_id=7,
            route_index=0,
            route_vertices=[
                CoordinateMsg(lat=p.lat, lon=p.lon, alt=p.alt) for p in route
            ],
        )
    )
    controller._target_received_s = NOW
    controller._traffic_aircraft = (
        TrafficAircraft(2, original.lat, original.lon, 10.0, 0.0, 0.0, 5.0, 5.0),
    )
    controller._traffic_sequence = 11
    controller._adjustment_pub = Mock()

    assert controller._next_setpoint(NOW) == HERE
    proposal = controller._adjustment_pub.publish.call_args.args[0]
    assert proposal.leg_id == 7
    assert proposal.route_index == 0
    assert proposal.traffic_sequence == 11
    assert proposal.replacement.lat > original.lat
    assert proposal.replacement.lon > original.lon
    controller._next_setpoint(NOW + 0.1)
    assert controller._adjustment_pub.publish.call_count == 1


def test_blocked_optional_lookahead_replans_to_required_waypoint(controller) -> None:
    controller._path = [HERE, TARGET, enu_to_coordinate(TARGET, 30.0, 0.0)]
    controller._target_index = 1
    optional_aim = controller._aim_point(HERE)
    assert optional_aim != TARGET
    controller._traffic_aircraft = (
        TrafficAircraft(
            2, optional_aim.lat, optional_aim.lon, 10.0, 0.0, 0.0, 2.0, 5.0
        ),
    )

    setpoint = controller._next_setpoint(NOW)

    assert setpoint != HERE
    assert controller._hold_reason is None


def test_snapshot_age_includes_time_before_ros_callback(controller) -> None:
    snapshot = TrafficSnapshot(connected=True, healthy=True)
    received_ns = controller.get_clock().now().nanoseconds - 2_000_000_000
    snapshot.header.stamp.sec = received_ns // 1_000_000_000
    snapshot.header.stamp.nanosec = received_ns % 1_000_000_000
    controller._traffic_callback(snapshot)

    assert time.monotonic() - controller._traffic_received_s == pytest.approx(
        2.0, abs=0.1
    )


def test_missing_receive_time_holds(controller) -> None:
    controller._traffic_callback(
        TrafficSnapshot(connected=True, healthy=True)
    )
    now_s = time.monotonic()
    controller._target_received_s = now_s
    place(controller, HERE, now_s)

    assert controller._next_setpoint(now_s) == HERE
    assert controller._hold_reason == "INVALID_TRAFFIC_RECEIVE_TIME"


def test_impossible_aircraft_jump_holds(controller) -> None:
    controller._traffic_tracks[2] = (
        time.monotonic() - 1.0,
        HERE.lat,
        HERE.lon,
        1.0,
        0.0,
    )
    jumped = enu_to_coordinate(HERE, 100.0, 0.0)
    snapshot = TrafficSnapshot(connected=True, healthy=True)
    snapshot.header.stamp = controller.get_clock().now().to_msg()
    snapshot.aircraft = [
        TrafficAircraftMsg(
            aircraft_index=2,
            latitude_deg=jumped.lat,
            longitude_deg=jumped.lon,
            altitude_agl_m=10.0,
            speed_mps=1.0,
            heading_deg_true=90.0,
            horizontal_keepaway_m=5.0,
            vertical_keepaway_m=5.0,
        )
    ]
    controller._traffic_callback(snapshot)
    now_s = time.monotonic()
    controller._target_received_s = now_s
    place(controller, HERE, now_s)

    assert controller._next_setpoint(now_s) == HERE
    assert controller._hold_reason == "INCONSISTENT_TRAFFIC_MOTION"


@pytest.mark.parametrize(
    ("reported_speed", "heading_deg", "actual_east_m"),
    [(0.0, 90.0, 1.2), (1.2, 0.0, 2.0)],
)
def test_speed_or_heading_inconsistency_holds(
    controller, reported_speed, heading_deg, actual_east_m
) -> None:
    controller._traffic_tracks[2] = (
        time.monotonic() - 1.0,
        HERE.lat,
        HERE.lon,
        reported_speed,
        heading_deg,
    )
    moved = enu_to_coordinate(HERE, actual_east_m, 0.0)
    snapshot = TrafficSnapshot(connected=True, healthy=True)
    snapshot.header.stamp = controller.get_clock().now().to_msg()
    snapshot.aircraft = [
        TrafficAircraftMsg(
            aircraft_index=2,
            latitude_deg=moved.lat,
            longitude_deg=moved.lon,
            altitude_agl_m=10.0,
            speed_mps=reported_speed,
            heading_deg_true=heading_deg,
            horizontal_keepaway_m=5.0,
            vertical_keepaway_m=5.0,
        )
    ]
    controller._traffic_callback(snapshot)
    now_s = time.monotonic()
    controller._target_received_s = now_s
    place(controller, HERE, now_s)

    assert controller._next_setpoint(now_s) == HERE
    assert controller._hold_reason == "INCONSISTENT_TRAFFIC_MOTION"


def test_persistent_blockage_holds_and_alerts_pilot(controller) -> None:
    assert controller._hold(HERE, "BLOCKED", NOW) == HERE
    assert controller._last_alert_s == -math.inf
    assert controller._hold(HERE, "BLOCKED", NOW + 5.0) == HERE
    assert controller._last_alert_s == NOW + 5.0


def test_predicted_hold_conflict_alerts_immediately(controller) -> None:
    assert controller._hold(HERE, "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED", NOW) == HERE
    assert controller._last_alert_s == NOW


@pytest.mark.parametrize(
    "reason",
    ["STALE_TRAFFIC", "TRAFFIC_SPEED_OUT_OF_ENVELOPE", "INCONSISTENT_TRAFFIC_MOTION"],
)
def test_untrusted_moving_traffic_alerts_immediately(controller, reason) -> None:
    assert controller._hold(HERE, reason, NOW) == HERE
    assert controller._last_alert_s == NOW


def test_crossing_yield_alerts_only_if_it_persists(controller) -> None:
    assert controller._hold(HERE, "YIELD_CROSSING", NOW) == HERE
    assert controller._last_alert_s == -math.inf
    assert controller._hold(HERE, "YIELD_CROSSING", NOW + 19.9) == HERE
    assert controller._last_alert_s == -math.inf
    assert controller._hold(HERE, "YIELD_CROSSING", NOW + 20.0) == HERE
    assert controller._last_alert_s == NOW + 20.0


def test_incoming_aircraft_index_is_treated_as_traffic(controller) -> None:
    controller._traffic_aircraft = (
        TrafficAircraft(2, HERE.lat, HERE.lon, 10.0, 0.0, 0.0, 2.0, 5.0),
    )
    assert controller._next_setpoint(NOW) == HERE
    assert controller._hold_reason is not None


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
