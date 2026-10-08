from __future__ import annotations

import math
from itertools import pairwise

import pytest
from engine.traffic_navigation import (
    TrafficAircraftState,
    TrafficNavigationConfig,
    TrafficSnapshotState,
    traffic_to_obstacle_snapshot,
)

NOW = 100.0
LATITUDE = 43.0
LONGITUDE = -80.0


def aircraft(**overrides: object) -> TrafficAircraftState:
    values: dict[str, object] = {
        "aircraft_index": 2,
        "name": "INTRUDER",
        "latitude_deg": LATITUDE,
        "longitude_deg": LONGITUDE,
        "altitude_agl_m": 10.0,
        "speed_mps": 0.0,
        "heading_deg_true": 0.0,
        "horizontal_keepaway_m": 2.0,
        "vertical_keepaway_m": 2.0,
    }
    values.update(overrides)
    return TrafficAircraftState(**values)  # type: ignore[arg-type]


def traffic(*items: TrafficAircraftState, **overrides: object) -> TrafficSnapshotState:
    values: dict[str, object] = {
        "sequence": 1,
        "connected": True,
        "healthy": True,
        "reason": "",
        "own_aircraft_index": 1,
        "aircraft": tuple(items),
        "received_s": NOW,
    }
    values.update(overrides)
    return TrafficSnapshotState(**values)  # type: ignore[arg-type]


def convert(snapshot: TrafficSnapshotState, **overrides: object):
    values: dict[str, object] = {
        "traffic": snapshot,
        "now_s": NOW,
        "vehicle_latitude_deg": LATITUDE,
        "vehicle_longitude_deg": LONGITUDE,
        "vehicle_east_m": 0.0,
        "vehicle_north_m": 0.0,
        "vehicle_altitude_agl_m": 10.0,
        "goal_altitude_agl_m": 10.0,
    }
    values.update(overrides)
    return traffic_to_obstacle_snapshot(**values)  # type: ignore[arg-type]


def test_fresh_empty_snapshot_is_clear_and_healthy() -> None:
    result = convert(traffic())

    assert result.reason is None
    assert result.snapshot.healthy
    assert result.snapshot.obstacles == ()
    assert result.raw_aircraft_count == 0


def test_repeated_stationary_snapshot_is_a_fresh_valid_update() -> None:
    stationary = aircraft(speed_mps=0.0)
    first = convert(traffic(stationary, sequence=1, received_s=NOW), now_s=NOW)
    repeated = convert(
        traffic(stationary, sequence=2, received_s=NOW + 1.0),
        now_s=NOW + 1.0,
    )

    assert first.reason is None
    assert repeated.reason is None
    assert repeated.snapshot.healthy
    assert repeated.snapshot.obstacles == first.snapshot.obstacles


@pytest.mark.parametrize(
    ("snapshot", "now_s", "reason"),
    [
        (traffic(connected=False, reason="DISCONNECTED"), NOW, "DISCONNECTED"),
        (traffic(healthy=False, reason="BAD_EVENT"), NOW, "BAD_EVENT"),
        (traffic(own_aircraft_index=-1), NOW, "UNVERIFIED_SELF_IDENTITY"),
        (traffic(received_s=NOW - 2.6), NOW, "STALE_TRAFFIC"),
        (traffic(received_s=NOW + 0.1), NOW, "FUTURE_TRAFFIC"),
    ],
)
def test_unhealthy_or_stale_input_fails_closed(
    snapshot: TrafficSnapshotState, now_s: float, reason: str
) -> None:
    result = convert(snapshot, now_s=now_s)

    assert result.reason == reason
    assert not result.snapshot.healthy
    assert result.snapshot.obstacles == ()


def test_self_is_filtered_but_other_aircraft_is_kept() -> None:
    result = convert(traffic(aircraft(aircraft_index=1), aircraft(aircraft_index=2)))

    assert result.reason is None
    assert result.raw_aircraft_count == 2
    assert result.relevant_aircraft_count == 1
    assert len(result.snapshot.obstacles) == 1


def test_altitude_interval_filters_irrelevant_aircraft() -> None:
    irrelevant = convert(traffic(aircraft(altitude_agl_m=30.0)))
    climbing_through = convert(
        traffic(aircraft(altitude_agl_m=20.0)),
        goal_altitude_agl_m=25.0,
    )

    assert irrelevant.relevant_aircraft_count == 0
    assert irrelevant.snapshot.obstacles == ()
    assert climbing_through.relevant_aircraft_count == 1


@pytest.mark.parametrize(
    ("heading_deg", "east_sign", "north_sign"),
    [
        (0.0, 0, 1),
        (90.0, 1, 0),
        (180.0, 0, -1),
        (270.0, -1, 0),
    ],
)
def test_true_heading_maps_to_enu_velocity(
    heading_deg: float, east_sign: int, north_sign: int
) -> None:
    result = convert(
        traffic(aircraft(speed_mps=1.0, heading_deg_true=heading_deg)),
        now_s=NOW + 1.0,
    )
    current = result.snapshot.obstacles[0].center

    if east_sign == 0:
        assert current.x == pytest.approx(0.0, abs=1e-9)
    else:
        assert math.copysign(1.0, current.x) == east_sign
    if north_sign == 0:
        assert current.y == pytest.approx(0.0, abs=1e-9)
    else:
        assert math.copysign(1.0, current.y) == north_sign


def test_one_hz_age_and_three_second_swept_corridor_are_both_predicted() -> None:
    result = convert(
        traffic(
            aircraft(
                speed_mps=2.0,
                heading_deg_true=90.0,
                horizontal_keepaway_m=1.0,
            )
        ),
        now_s=NOW + 1.0,
    )

    centers = [item.center.x for item in result.snapshot.obstacles]
    assert centers[0] == pytest.approx(2.0)
    assert centers[-1] == pytest.approx(8.0)
    assert max(b - a for a, b in pairwise(centers)) <= 1.0
    assert all(
        item.radius_m == pytest.approx(1.0) for item in result.snapshot.obstacles
    )


def test_obstacle_budget_overflow_fails_closed() -> None:
    result = convert(
        traffic(
            aircraft(
                speed_mps=10.0,
                horizontal_keepaway_m=0.1,
            )
        ),
        config=TrafficNavigationConfig(maximum_predicted_obstacles=10),
    )

    assert result.reason == "TOO_MANY_PREDICTED_OBSTACLES"
    assert not result.snapshot.healthy


def test_duplicate_identity_fails_closed_defensively() -> None:
    result = convert(traffic(aircraft(), aircraft(name="DUPLICATE")))

    assert result.reason == "DUPLICATE_TRAFFIC_IDENTITY"
    assert not result.snapshot.healthy
