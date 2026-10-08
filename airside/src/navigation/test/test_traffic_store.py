from navigation.traffic_store import (
    ObstacleRecord,
    TrafficStore,
    altitude_ranges_overlap,
)


def obstacle(sequence: int, index: int) -> ObstacleRecord:
    return ObstacleRecord(
        sequence=sequence,
        aircraft_index=index,
        latitude_deg=43.47,
        longitude_deg=-80.54,
        altitude_agl_m=15.0,
        horizontal_keep_away_m=5.0,
        vertical_keep_away_m=3.0,
        speed_mps=0.0,
        direction_deg=0.0,
    )


def status(store: TrafficStore, sequence: int, indices: tuple[int, ...]) -> None:
    store.receive_status(
        sequence=sequence,
        connected=True,
        healthy=True,
        reason="",
        aircraft_indices=indices,
        received_s=10.0,
    )


def test_commits_only_after_complete_snapshot() -> None:
    store = TrafficStore(own_aircraft_index=1, freshness_s=2.5)
    store.receive_obstacle(obstacle(4, 2))
    status(store, 4, (2, 3))
    assert store.current(10.1) == ((), "INCOMPLETE_TRAFFIC_SNAPSHOT")

    store.receive_obstacle(obstacle(4, 3))
    current, reason = store.current(10.1)
    assert reason is None
    assert [item.aircraft_index for item in current] == [2, 3]


def test_commits_when_status_arrives_before_obstacles() -> None:
    store = TrafficStore(own_aircraft_index=1, freshness_s=2.5)
    status(store, 4, (2, 3))
    assert store.current(10.1) == ((), "INCOMPLETE_TRAFFIC_SNAPSHOT")

    store.receive_obstacle(obstacle(4, 3))
    store.receive_obstacle(obstacle(4, 2))
    current, reason = store.current(10.1)
    assert reason is None
    assert [item.aircraft_index for item in current] == [2, 3]


def test_empty_snapshot_clears_previous_aircraft() -> None:
    store = TrafficStore(own_aircraft_index=1, freshness_s=2.5)
    store.receive_obstacle(obstacle(1, 2))
    status(store, 1, (2,))
    status(store, 2, ())

    assert store.current(10.1) == ((), None)


def test_new_snapshot_removes_disappeared_aircraft() -> None:
    store = TrafficStore(own_aircraft_index=1, freshness_s=2.5)
    store.receive_obstacle(obstacle(1, 2))
    store.receive_obstacle(obstacle(1, 3))
    status(store, 1, (2, 3))

    store.receive_obstacle(obstacle(2, 3))
    status(store, 2, (3,))
    current, reason = store.current(10.1)

    assert reason is None
    assert [item.aircraft_index for item in current] == [3]


def test_unhealthy_and_stale_snapshots_fail_closed() -> None:
    store = TrafficStore(own_aircraft_index=1, freshness_s=2.5)
    store.receive_status(
        sequence=1,
        connected=False,
        healthy=False,
        reason="AEAC_DISCONNECTED",
        aircraft_indices=(),
        received_s=10.0,
    )
    assert store.current(10.1) == ((), "AEAC_DISCONNECTED")

    status(store, 2, ())
    assert store.current(12.6) == ((), "STALE_TRAFFIC")


def test_own_aircraft_in_snapshot_is_rejected() -> None:
    store = TrafficStore(own_aircraft_index=1, freshness_s=2.5)
    status(store, 1, (1,))

    assert store.current(10.1) == ((), "OWN_AIRCRAFT_NOT_FILTERED")


def test_vertical_keepaway_filters_unrelated_altitude() -> None:
    assert altitude_ranges_overlap(
        vehicle_altitude_agl_m=10.0,
        target_altitude_agl_m=20.0,
        obstacle_altitude_agl_m=17.0,
        vertical_keep_away_m=3.0,
    )
    assert not altitude_ranges_overlap(
        vehicle_altitude_agl_m=10.0,
        target_altitude_agl_m=12.0,
        obstacle_altitude_agl_m=30.0,
        vertical_keep_away_m=3.0,
    )
