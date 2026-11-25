import math
from itertools import pairwise

import pytest

from obstacle_avoidance import TrafficAircraft, traffic_to_obstacles

LAT = 43.0
LON = -80.0


def aircraft(**overrides: object) -> TrafficAircraft:
    values: dict[str, object] = {
        "aircraft_index": 2,
        "lat": LAT,
        "lon": LON,
        "altitude_agl_m": 10.0,
        "speed_mps": 0.0,
        "heading_deg": 0.0,
        "horizontal_keep_away_m": 2.0,
        "vertical_keep_away_m": 2.0,
    }
    values.update(overrides)
    return TrafficAircraft(**values)  # type: ignore[arg-type]


def convert(*traffic: TrafficAircraft, **overrides: object):
    values: dict[str, object] = {
        "origin_lat": LAT,
        "origin_lon": LON,
        "altitude_band_m": (10.0, 10.0),
        "age_s": 0.0,
        "prediction_horizon_s": 3.0,
        "maximum_step_m": 1.0,
        "maximum_obstacles": 512,
    }
    values.update(overrides)
    return traffic_to_obstacles(tuple(traffic), **values)  # type: ignore[arg-type]


def test_no_traffic_is_no_obstacles() -> None:
    assert convert() == ()


def test_altitude_band_filters_irrelevant_aircraft() -> None:
    assert convert(aircraft(altitude_agl_m=30.0)) == ()
    assert len(convert(aircraft(altitude_agl_m=20.0), altitude_band_m=(10.0, 25.0))) == 1


@pytest.mark.parametrize(
    ("heading_deg", "east_sign", "north_sign"),
    [(0.0, 0, 1), (90.0, 1, 0), (180.0, 0, -1), (270.0, -1, 0)],
)
def test_true_heading_maps_to_enu(heading_deg: float, east_sign: int, north_sign: int) -> None:
    current = convert(aircraft(speed_mps=1.0, heading_deg=heading_deg), age_s=1.0)[0].center

    for value, sign in ((current.x, east_sign), (current.y, north_sign)):
        if sign == 0:
            assert value == pytest.approx(0.0, abs=1e-9)
        else:
            assert math.copysign(1.0, value) == sign


def test_age_and_swept_corridor_are_both_predicted() -> None:
    obstacles = convert(
        aircraft(speed_mps=2.0, heading_deg=90.0, horizontal_keep_away_m=1.0),
        age_s=1.0,
    )

    centers = [item.center.x for item in obstacles]
    assert centers[0] == pytest.approx(2.0)
    assert centers[-1] == pytest.approx(8.0)
    assert max(b - a for a, b in pairwise(centers)) <= 1.0
    assert all(item.radius_m == pytest.approx(1.0) for item in obstacles)


@pytest.mark.parametrize(
    ("traffic", "overrides", "reason"),
    [
        ((aircraft(), aircraft()), {}, "DUPLICATE_TRAFFIC_IDENTITY"),
        ((aircraft(speed_mps=-1.0),), {}, "INVALID_TRAFFIC_AIRCRAFT"),
        ((aircraft(lat=math.nan),), {}, "INVALID_TRAFFIC_AIRCRAFT"),
        (
            (aircraft(speed_mps=10.0, horizontal_keep_away_m=0.1),),
            {"maximum_obstacles": 10},
            "TOO_MANY_PREDICTED_OBSTACLES",
        ),
    ],
)
def test_untrustworthy_traffic_raises(traffic, overrides, reason) -> None:
    with pytest.raises(ValueError, match=reason):
        convert(*traffic, **overrides)
