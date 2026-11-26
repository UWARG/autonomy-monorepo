"""The mission manager must own and validate a substituted lap target."""

import json
from unittest.mock import Mock

from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.msg import PositionTarget, WaypointAdjustment
from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from obstacle_avoidance.boundary import FlightBoundary
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate


def test_only_matching_leg_may_replace_lap_target() -> None:
    home = Coordinate(43.0, -80.0, 10.0)
    route = [
        enu_to_coordinate(home, east, north, 10.0)
        for east, north in ((10, 10), (10, -10), (-10, -10), (-10, 10))
    ]
    corners = [
        enu_to_coordinate(home, east, north)
        for east, north in ((-60, -60), (60, -60), (60, 60), (-60, 60))
    ]
    node = FlyToWaypoint(fly_through=True)
    node._node = Mock()
    node._boundary = FlightBoundary.from_json(
        json.dumps([[p.lat, p.lon] for p in corners])
    )
    node._original_waypoint = CoordinateMsg(
        lat=route[0].lat, lon=route[0].lon, alt=route[0].alt
    )
    node._replacement_active = False
    node._last_adjustment_sequence = 0
    original = node._original_waypoint
    node._target = PositionTarget(
        target=original,
        path=[original],
        target_index=0,
        leg_id=3,
        route_index=0,
        route_vertices=[CoordinateMsg(lat=p.lat, lon=p.lon, alt=p.alt) for p in route],
    )
    replacement = enu_to_coordinate(route[0], 10, 10, 10.0)
    proposal = WaypointAdjustment(
        leg_id=2,
        route_index=0,
        original=original,
        replacement=CoordinateMsg(
            lat=replacement.lat, lon=replacement.lon, alt=replacement.alt
        ),
        traffic_sequence=5,
    )
    node._adjustment_callback(proposal)
    assert node._target.target == original

    proposal.leg_id = 3
    node._adjustment_callback(proposal)
    assert node._replacement_active
    assert node._target.target == proposal.replacement
    assert node._target.path[0] == proposal.replacement
    assert node._last_adjustment_sequence == 5

    proposal.traffic_sequence = 4
    node._adjustment_callback(proposal)
    assert node._last_adjustment_sequence == 5
