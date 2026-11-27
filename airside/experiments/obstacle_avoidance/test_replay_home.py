"""The capture replay must retain world coordinates when home is off route."""

import math
from types import SimpleNamespace

from server_traffic_replay import east_north
from sitl_moving_readiness import offset
from sitl_server_replay import ReplayScenario


def test_replay_waypoints_stay_at_wgs84_coordinates_with_separate_home() -> None:
    home = (43.4338803, -80.5775947)
    route = (
        (43.433915, -80.577225),
        (43.434191, -80.578572),
        (43.433813, -80.577665),
    )
    scenario = SimpleNamespace(route=route)

    ReplayScenario.set_home_origin(scenario, home)

    assert scenario.origin == home
    assert math.hypot(*scenario.route_local[0]) > 5.0
    for original, local in zip(route, scenario.route_local):
        recovered = offset(*home, *local)
        assert math.hypot(*east_north(original, recovered)) < 0.1
