import math
from pathlib import Path

from engine.fence import RECOVERY_MARGIN_M, Fence

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import parse_geofence_file

FENCE = Fence(
    [
        Coordinate(43.438391, -80.571419, 0.0),
        Coordinate(43.438391, -80.583781, 0.0),
        Coordinate(43.429409, -80.583781, 0.0),
        Coordinate(43.429409, -80.571419, 0.0),
    ]
)


def at(east_m: float, north_m: float, alt_m: float = 10.0) -> Coordinate:
    """A coordinate at the given local east/north offset from the fence anchor."""
    return enu_to_coordinate(FENCE.anchor, east_m, north_m, alt_m)


def test_configured_fence_loads():
    path = Path(__file__).parents[1] / "config" / "geofence.yaml"
    fence = Fence(parse_geofence_file(path))
    assert len(fence.vertices) == 8
    assert fence.contains(Coordinate(43.4339, -80.5776, 10.0))
    # The east-edge notch is outside; the same latitude west of it is inside.
    assert not fence.contains(Coordinate(43.4350, -80.5730, 10.0))
    assert fence.contains(Coordinate(43.4350, -80.5765, 10.0))
    assert not fence.contains(Coordinate(0.0, 0.0, 10.0))


def test_contains_inside():
    assert FENCE.contains(at(0.0, 0.0))
    assert FENCE.contains(at(400.0, -400.0))


def test_contains_outside():
    assert not FENCE.contains(at(2000.0, 0.0))
    assert not FENCE.contains(at(0.0, -2000.0))
    assert not FENCE.contains(Coordinate(0.0, 0.0, 10.0))


def test_clearance_is_positive_inside_and_negative_outside():
    assert FENCE.clearance_m(at(0.0, 0.0)) > 100.0
    assert FENCE.clearance_m(at(2000.0, 0.0)) < 0.0


def test_time_to_breach_when_closing():
    # 300m from the anchor heading east leaves roughly 200m to the east edge.
    time_to_breach_s = FENCE.time_to_breach_s(at(300.0, 0.0), (10.0, 0.0, 0.0))
    assert 15.0 < time_to_breach_s < 25.0


def test_time_to_breach_is_infinite_when_stationary():
    assert math.isinf(FENCE.time_to_breach_s(at(0.0, 0.0), (0.0, 0.0, 0.0)))


def test_time_to_breach_ignores_slow_drift_and_tangential_noise():
    assert math.isinf(FENCE.time_to_breach_s(at(450.0, 0.0), (0.05, 0.0, 0.0)))
    assert math.isinf(FENCE.time_to_breach_s(at(450.0, 0.0), (0.0, 0.2, 0.0)))


def test_time_to_breach_is_zero_when_already_outside():
    assert FENCE.time_to_breach_s(at(2000.0, 0.0), (10.0, 0.0, 0.0)) == 0.0


def test_breached_only_when_outside():
    assert FENCE.breached(at(2000.0, 0.0))
    assert not FENCE.breached(at(450.0, 0.0))
    assert not FENCE.breached(at(460.0, 0.0))
    assert not FENCE.breached(at(0.0, 0.0))


def test_near_the_edge_warns_without_breaching():
    velocity = (0.0, 0.0, 0.0)
    # 460m east leaves about 40m to the edge: inside the warning band.
    assert FENCE.near(at(460.0, 0.0), velocity)
    assert not FENCE.breached(at(460.0, 0.0))
    assert FENCE.near(at(2000.0, 0.0), velocity)


def test_centre_is_clear():
    velocity = (0.0, 0.0, 0.0)
    assert not FENCE.near(at(0.0, 0.0), velocity)
    assert not FENCE.breached(at(0.0, 0.0))


def test_recovery_target_is_inside_the_recovery_margin():
    target = FENCE.recovery_coordinate(at(2000.0, 0.0))
    assert FENCE.contains(target)
    assert FENCE.clearance_m(target) >= RECOVERY_MARGIN_M - 0.5


def test_recovery_target_keeps_altitude():
    assert FENCE.recovery_coordinate(at(2000.0, 0.0, 25.0)).alt == 25.0
