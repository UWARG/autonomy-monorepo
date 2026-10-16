"""Tests for the groundside drone task scheduler."""

import math
import threading
import time

import pytest

from drone_task_scheduler import DroneTaskScheduler, Mission
from groundside_manager import DroneState, MissionState
from utils import Coordinate

BASE_LAT, BASE_LON, BASE_ALT = 43.4717, -80.5414, 15.0
M_PER_DEG_LAT = 111_000.0
M_PER_DEG_LON = M_PER_DEG_LAT * math.cos(math.radians(BASE_LAT))


def north(metres: float) -> Coordinate:
    """A Coordinate `metres` due north of the base point."""
    return Coordinate(BASE_LAT + metres / M_PER_DEG_LAT, BASE_LON, BASE_ALT)


def east(metres: float) -> Coordinate:
    """A Coordinate `metres` due east of the base point."""
    return Coordinate(BASE_LAT, BASE_LON + metres / M_PER_DEG_LON, BASE_ALT)


def make_state(drone_id, position, *, age_s=0.0) -> DroneState:
    """A DroneState whose last update is `age_s` seconds old."""
    return DroneState(
        drone_id=drone_id,
        vehicle_type="QUADCOPTER",
        position=position,
        orientation=None,
        mission_state=MissionState.IDLE,
        active_task_id=None,
        last_update_timestamp=time.time() - age_s,
        connected=True,
    )


def make_mission(task_id="task-1", target=None, yaw=1.57) -> Mission:
    return Mission(task_id, target or north(0.0), yaw)


class FakeDroneManager:
    """Duck-types the two DroneStateManager methods the scheduler calls.

    Mirrors real availability semantics: a drone that accepts a command stops
    being returned by get_available_drones() until its next DRONE_STATE reset.
    """

    def __init__(self, states=(), *, accept=True, raise_on_send=False):
        self._states = list(states)
        self.accept = accept
        self.raise_on_send = raise_on_send
        self.sent = []

    def get_available_drones(self):
        return list(self._states)

    def send_location_command_sync(self, drone_id, task_id, target, yaw):
        if self.raise_on_send:
            raise ConnectionResetError("socket closed")
        self.sent.append({
            "drone_id": drone_id,
            "task_id": task_id,
            "target": target,
            "yaw": yaw,
        })
        if self.accept:
            self._states = [s for s in self._states if s.drone_id != drone_id]
        return self.accept


@pytest.fixture
def fast_poll():
    """Shrink the poll interval so threaded tests finish in milliseconds."""
    def _apply(scheduler):
        scheduler.POLLING_HERTZ = 50.0  # 20 ms per pass
    return _apply


def run_passes(scheduler, manager, expected_sends, timeout=2.0):
    """Start the scheduler, wait for `expected_sends` commands, then stop."""
    scheduler.start()
    deadline = time.time() + timeout
    try:
        while len(manager.sent) < expected_sends and time.time() < deadline:
            time.sleep(0.005)
    finally:
        scheduler.stop()


# ---------------------------------------------------------------------------
# get_proximity - units, symmetry, altitude
# ---------------------------------------------------------------------------

def test_proximity_is_symmetric_for_equal_north_and_east_offsets():
    """Regression: a degree of latitude is not a degree of longitude, so a
    naive 3D norm over (lat, lon, alt) misranks due-east against due-north."""
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.get_proximity(north(10), north(0)) == pytest.approx(
        scheduler.get_proximity(east(10), north(0)), abs=0.5)


def test_proximity_matches_known_distance():
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.get_proximity(north(25), north(0)) == pytest.approx(
        25.0, abs=0.5)


def test_proximity_includes_altitude():
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    climbed = Coordinate(BASE_LAT, BASE_LON, BASE_ALT + 30)
    assert scheduler.get_proximity(climbed, north(0)) == pytest.approx(
        30.0, abs=0.5)


def test_proximity_is_zero_at_same_point():
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.get_proximity(north(0), north(0)) == pytest.approx(
        0.0, abs=1e-6)


# ---------------------------------------------------------------------------
# assign_mission - selection, exclusions, per-pass drone usage
# ---------------------------------------------------------------------------

def test_assigns_to_closest_drone():
    near = make_state("drone-near", north(10))
    far = make_state("drone-far", north(80))
    scheduler = DroneTaskScheduler([], FakeDroneManager([near, far]))
    assert scheduler.assign_mission(make_mission(), [near, far], set()) == "drone-near"


def test_assigns_one_mission_per_drone_then_reports_exhausted():
    """The anti-hoarding rule: a drone already given work this pass is
    skipped, and once all are taken the mission stays queued."""
    near = make_state("drone-near", north(10))
    far = make_state("drone-far", north(80))
    scheduler = DroneTaskScheduler([], FakeDroneManager([near, far]))
    mission = make_mission()

    assert scheduler.assign_mission(mission, [near, far], set()) == "drone-near"
    assert scheduler.assign_mission(mission, [near, far], {"drone-near"}) == "drone-far"
    assert scheduler.assign_mission(
        mission, [near, far], {"drone-near", "drone-far"}) is None


def test_ties_break_deterministically_on_first_drone():
    a = make_state("drone-a", north(10))
    b = make_state("drone-b", north(10))
    scheduler = DroneTaskScheduler([], FakeDroneManager([a, b]))
    assert scheduler.assign_mission(make_mission(), [a, b], set()) == "drone-a"


def test_skips_drone_with_no_position():
    positioned = make_state("drone-ok", north(10))
    blind = make_state("drone-blind", None)
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.assign_mission(
        make_mission(), [positioned, blind], set()) == "drone-ok"


def test_skips_drone_with_no_timestamp():
    positioned = make_state("drone-ok", north(10))
    blind = make_state("drone-blind", north(5))
    blind.last_update_timestamp = None
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.assign_mission(
        make_mission(), [positioned, blind], set()) == "drone-ok"


def test_skips_stale_drone():
    """Pins the staleness contract: last_update_timestamp is a time.time()
    value, so a stale drone loses to a fresh one regardless of distance."""
    stale = make_state("drone-stale", north(1), age_s=3600)
    fresh = make_state("drone-fresh", north(90))
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.assign_mission(make_mission(), [stale, fresh], set()) == "drone-fresh"


def test_returns_none_when_no_drones_available():
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    assert scheduler.assign_mission(make_mission(), [], set()) is None


# ---------------------------------------------------------------------------
# Command payload contract
# ---------------------------------------------------------------------------

def test_command_carries_drone_id_task_id_target_and_yaw():
    drone = make_state("drone-03", north(10))
    manager = FakeDroneManager([drone])
    scheduler = DroneTaskScheduler([], manager)
    target, yaw = north(42), 1.57

    scheduler.assign_mission(make_mission("task-17", target, yaw), [drone], set())

    assert manager.sent == [{
        "drone_id": "drone-03",
        "task_id": "task-17",
        "target": target,
        "yaw": yaw,
    }]


# ---------------------------------------------------------------------------
# Scheduling loop - queue draining (threaded)
# ---------------------------------------------------------------------------

def test_drains_queue_across_multiple_drones(fast_poll):
    states = [make_state(f"drone-{i:02d}", north(i * 10)) for i in range(3)]
    manager = FakeDroneManager(states)
    scheduler = DroneTaskScheduler(
        [make_mission(f"task-{i}", north(i * 10)) for i in range(3)], manager)
    fast_poll(scheduler)

    run_passes(scheduler, manager, expected_sends=3)

    assert len(manager.sent) == 3
    assert len({command["drone_id"] for command in manager.sent}) == 3
    assert scheduler.missions == []


def test_keeps_missions_queued_when_drones_are_short(fast_poll):
    manager = FakeDroneManager([make_state("drone-00", north(0))])
    scheduler = DroneTaskScheduler(
        [make_mission(f"task-{i}") for i in range(3)], manager)
    fast_poll(scheduler)

    run_passes(scheduler, manager, expected_sends=1)
    time.sleep(0.05)

    assert len(manager.sent) == 1
    assert len(scheduler.missions) == 2


def test_rejected_mission_is_retained_for_retry(fast_poll):
    manager = FakeDroneManager([make_state("drone-00", north(0))], accept=False)
    scheduler = DroneTaskScheduler([make_mission("task-1")], manager)
    fast_poll(scheduler)

    scheduler.start()
    time.sleep(0.1)
    scheduler.stop()

    assert manager.sent, "the command should still have been attempted"
    assert len(scheduler.missions) == 1


def test_survives_a_send_failure_without_dying(fast_poll):
    manager = FakeDroneManager(
        [make_state("drone-00", north(0))], raise_on_send=True)
    scheduler = DroneTaskScheduler([make_mission("task-1")], manager)
    fast_poll(scheduler)

    scheduler.start()
    time.sleep(0.1)
    alive = scheduler.polling_thread.is_alive()
    scheduler.stop()

    assert alive, "one failed send must not kill the polling thread"
    assert len(scheduler.missions) == 1


# ---------------------------------------------------------------------------
# Concurrency and lifecycle
# ---------------------------------------------------------------------------

def test_concurrent_add_mission_loses_nothing(fast_poll):
    manager = FakeDroneManager([make_state("drone-00", north(0))], accept=False)
    scheduler = DroneTaskScheduler([], manager)
    fast_poll(scheduler)
    scheduler.start()

    added = []

    def writer(prefix):
        for index in range(100):
            task_id = f"{prefix}-{index}"
            scheduler.add_mission(make_mission(task_id))
            added.append(task_id)

    threads = [threading.Thread(target=writer, args=(f"w{n}",)) for n in range(3)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    time.sleep(0.05)
    scheduler.stop()

    assert len(scheduler.missions) == len(added)


def test_start_and_stop_are_idempotent(fast_poll):
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    fast_poll(scheduler)

    scheduler.start()
    scheduler.start()  # must stop the previous thread first
    time.sleep(0.05)
    scheduler.stop()
    scheduler.stop()

    assert not scheduler.polling_thread.is_alive()


def test_stop_before_start_is_safe():
    scheduler = DroneTaskScheduler([], FakeDroneManager())
    scheduler.stop()  # must not raise