"""
Unit tests for route-preserving dispatch of planned Missions.
"""

from __future__ import annotations

import random
from collections import Counter

import pytest
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState, MissionState
from src.survey_executive.dispatch import DroneManagerLike, RouteDispatcher
from tests.fakes import FakeClock
from tests.unit.survey_executive.factories import LAT0, LON0
from tests.unit.survey_executive.fakes import FakeFleet

TIMEOUT_S = 3.0
DROPOUT_S = 10.0


def route(drone_id: str, count: int, round_index: int = 0) -> list[Mission]:
    return [
        Mission(
            task_id=f"s-r{round_index}-{drone_id}-{index:03d}",
            target=Coordinate(LAT0 + index * 1e-5, LON0, 30.0),
            yaw=0.5,
        )
        for index in range(count)
    ]


def make_dispatcher(fleet: FakeFleet) -> RouteDispatcher:
    return RouteDispatcher(
        fleet, fleet.clock, ack_timeout_s=TIMEOUT_S, dropout_after_s=DROPOUT_S
    )


def test_fake_fleet_matches_the_protocol() -> None:
    manager: DroneManagerLike = FakeFleet("a")
    assert manager.get_all_drones()["a"].drone_id == "a"


def test_each_drone_flies_its_route_in_planned_order() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    routes = {"a": route("a", 3), "b": route("b", 2)}
    dispatcher.load(routes)

    for _ in range(10):
        dispatcher.tick()
        for drone_id in ("a", "b"):
            mission = dispatcher.in_flight(drone_id)
            if mission is not None:
                fleet.fly(drone_id, mission.task_id)

    for drone_id, planned in routes.items():
        flown = [task for sent_to, task in fleet.sent if sent_to == drone_id]
        assert flown == [mission.task_id for mission in planned]
        finished = [task for done_by, task in dispatcher.completed() if done_by == drone_id]
        assert finished == flown
    assert dispatcher.is_done()


def test_sends_one_mission_at_a_time() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 3)})

    dispatcher.tick()
    fleet.report("a", MissionState.MOVING, "s-r0-a-000")
    dispatcher.tick()
    dispatcher.tick()

    assert fleet.sent == [("a", "s-r0-a-000")]
    assert [m.task_id for m in dispatcher.remaining("a")] == [
        "s-r0-a-000",
        "s-r0-a-001",
        "s-r0-a-002",
    ]


def test_next_mission_goes_out_on_the_tick_that_sees_arrival() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2)})
    dispatcher.tick()
    fleet.fly("a", "s-r0-a-000")

    finished = dispatcher.tick()

    assert finished == [("a", "s-r0-a-000")]
    assert fleet.sent[-1] == ("a", "s-r0-a-001")


def test_arrived_on_the_previous_task_does_not_finish_the_new_one() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2)})
    dispatcher.tick()
    fleet.fly("a", "s-r0-a-000")
    dispatcher.tick()  # finishes 000, sends 001

    # The drone has not reported since: still ARRIVED on 000.
    assert dispatcher.tick() == []
    assert dispatcher.in_flight("a").task_id == "s-r0-a-001"


def test_waits_for_a_drone_moving_on_someone_elses_task() -> None:
    fleet = FakeFleet("a")
    fleet.report("a", MissionState.MOVING, "other-task")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})

    dispatcher.tick()
    assert fleet.sent == []

    fleet.report("a", MissionState.ARRIVED, "other-task")
    dispatcher.tick()
    assert fleet.sent == [("a", "s-r0-a-000")]


def test_resends_a_turned_down_command_only_after_the_timeout() -> None:
    fleet = FakeFleet("a")
    clock = fleet.clock
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()
    # The drone turned it down: it never reports our task.

    clock.advance(TIMEOUT_S - 0.1)
    dispatcher.tick()
    assert fleet.sent == [("a", "s-r0-a-000")]

    clock.advance(0.1)
    dispatcher.tick()
    assert fleet.sent == [("a", "s-r0-a-000")] * 2

    # The timeout restarts from the resend.
    clock.advance(TIMEOUT_S - 0.1)
    dispatcher.tick()
    assert len(fleet.sent) == 2


def test_no_resend_once_the_drone_reports_the_task() -> None:
    fleet = FakeFleet("a")
    clock = fleet.clock
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()
    fleet.report("a", MissionState.MOVING, "s-r0-a-000")
    dispatcher.tick()

    clock.advance(10 * TIMEOUT_S)
    dispatcher.tick()

    assert len(fleet.sent) == 1


def test_resends_when_a_drone_loses_a_task_it_had_taken() -> None:
    fleet = FakeFleet("a")
    clock = fleet.clock
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()
    clock.advance(1.0)
    fleet.report("a", MissionState.MOVING, "s-r0-a-000")
    dispatcher.tick()

    # Restarted: back to IDLE with no task.
    fleet.report("a", MissionState.IDLE, None)
    clock.advance(TIMEOUT_S - 0.1)
    dispatcher.tick()
    assert len(fleet.sent) == 1

    clock.advance(0.1)
    dispatcher.tick()
    assert fleet.sent == [("a", "s-r0-a-000")] * 2


def test_failed_send_keeps_the_mission_at_the_head() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2)})
    fleet.states["a"].connected = False

    dispatcher.tick()
    assert fleet.sent == []
    assert dispatcher.in_flight("a") is None

    fleet.states["a"].connected = True
    dispatcher.tick()
    assert fleet.sent == [("a", "s-r0-a-000")]


def test_failed_resend_is_tried_again_next_tick() -> None:
    class FlakyFleet(FakeFleet):
        def __init__(self) -> None:
            super().__init__("a")
            self.fail_next = False

        def send_location_command_sync(self, drone_id, task_id, target, yaw):  # type: ignore[no-untyped-def]
            if self.fail_next:
                self.fail_next = False
                return False
            return super().send_location_command_sync(drone_id, task_id, target, yaw)

    fleet = FlakyFleet()
    clock = fleet.clock
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()

    clock.advance(TIMEOUT_S)
    fleet.fail_next = True
    dispatcher.tick()
    assert len(fleet.sent) == 1

    dispatcher.tick()
    assert len(fleet.sent) == 2


def test_disconnected_drone_is_skipped_without_losing_missions() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2), "b": route("b", 1)})
    dispatcher.tick()
    fleet.states["a"].connected = False
    fleet.fly("b", "s-r0-b-000")

    assert dispatcher.tick() == [("b", "s-r0-b-000")]
    assert [m.task_id for m in dispatcher.remaining("a")] == [
        "s-r0-a-000",
        "s-r0-a-001",
    ]
    assert not dispatcher.is_done()


def test_unknown_drone_is_skipped() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"ghost": route("ghost", 1)})

    assert dispatcher.tick() == []
    assert fleet.sent == []


def test_repeated_arrived_reports_finish_a_mission_once() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()
    fleet.fly("a", "s-r0-a-000")

    for _ in range(5):
        dispatcher.tick()

    assert dispatcher.completed() == [("a", "s-r0-a-000")]
    assert dispatcher.is_done()


def test_load_during_a_run_queues_after_existing_missions() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2)})
    dispatcher.tick()

    dispatcher.load({"a": route("a", 1, round_index=1)})

    assert [m.task_id for m in dispatcher.remaining("a")] == [
        "s-r0-a-000",
        "s-r0-a-001",
        "s-r1-a-000",
    ]


def test_load_after_finishing_starts_again() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()
    fleet.fly("a", "s-r0-a-000")
    dispatcher.tick()
    assert dispatcher.is_done()

    dispatcher.load({"a": route("a", 1, round_index=1)})
    dispatcher.tick()

    assert not dispatcher.is_done()
    assert fleet.sent[-1] == ("a", "s-r1-a-000")


def test_duplicate_task_ids_are_refused_before_queueing() -> None:
    dispatcher = make_dispatcher(FakeFleet("a"))
    dispatcher.load({"a": route("a", 1)})

    with pytest.raises(ValueError, match="duplicate"):
        dispatcher.load({"a": route("a", 2)})
    with pytest.raises(ValueError, match="duplicate"):
        dispatcher.load({"b": route("a", 1, round_index=1) * 2})
    assert [m.task_id for m in dispatcher.remaining("a")] == ["s-r0-a-000"]
    assert dispatcher.remaining("b") == []


def test_empty_routes_are_done_straight_away() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": []})

    assert dispatcher.tick() == []
    assert dispatcher.is_done()
    assert fleet.sent == []


def test_remaining_is_a_copy() -> None:
    dispatcher = make_dispatcher(FakeFleet("a"))
    dispatcher.load({"a": route("a", 2)})

    dispatcher.remaining("a").clear()

    assert len(dispatcher.remaining("a")) == 2


@pytest.mark.parametrize("timeout_s", [0.0, -1.0])
def test_non_positive_timeout_is_refused(timeout_s: float) -> None:
    with pytest.raises(ValueError, match="ack_timeout_s"):
        RouteDispatcher(FakeFleet(), FakeClock(), ack_timeout_s=timeout_s)


def ids(missions: list[Mission]) -> list[str]:
    return [mission.task_id for mission in missions]


def test_dropped_drone_hands_its_unfinished_route_to_a_survivor() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 3), "b": route("b", 1)})
    dispatcher.tick()
    fleet.fly("b", "s-r0-b-000")
    fleet.kill("a")  # mid-leg on a-000

    fleet.clock.advance(DROPOUT_S)
    assert dispatcher.tick() == [("b", "s-r0-b-000")]

    assert dispatcher.dropped() == {"a"}
    assert dispatcher.remaining("a") == []
    assert ids(dispatcher.remaining("b")) == [
        "s-r0-a-000",
        "s-r0-a-001",
        "s-r0-a-002",
    ]
    assert fleet.sent[-1] == ("b", "s-r0-a-000")


def test_short_silence_does_not_drop_a_drone() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2), "b": route("b", 1)})
    dispatcher.tick()
    fleet.kill("a")

    fleet.clock.advance(DROPOUT_S - 0.1)
    dispatcher.tick()
    fleet.revive("a")
    dispatcher.tick()
    fleet.clock.advance(DROPOUT_S - 0.1)
    dispatcher.tick()

    assert dispatcher.dropped() == set()
    assert ids(dispatcher.remaining("a")) == ["s-r0-a-000", "s-r0-a-001"]


def test_missions_wait_when_no_drone_is_left() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2)})
    dispatcher.tick()
    fleet.kill("a")
    fleet.clock.advance(DROPOUT_S)
    dispatcher.tick()

    assert ids(dispatcher.unassigned()) == ["s-r0-a-000", "s-r0-a-001"]
    assert not dispatcher.is_done()

    fleet.revive("a")
    dispatcher.tick()

    assert dispatcher.dropped() == set()
    assert dispatcher.unassigned() == []
    assert fleet.sent[-1] == ("a", "s-r0-a-000")


def test_drone_that_never_reports_is_dropped_after_the_grace_period() -> None:
    fleet = FakeFleet("a")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": [], "ghost": route("ghost", 1)})

    fleet.clock.advance(DROPOUT_S - 0.1)
    dispatcher.tick()
    assert dispatcher.dropped() == set()

    fleet.clock.advance(0.1)
    dispatcher.tick()
    assert dispatcher.dropped() == {"ghost"}
    assert fleet.sent == [("a", "s-r0-ghost-000")]


def test_rejoined_drone_does_not_get_its_old_missions_back() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2), "b": []})
    dispatcher.tick()
    fleet.kill("a")
    fleet.clock.advance(DROPOUT_S)
    dispatcher.tick()

    fleet.revive("a")
    dispatcher.tick()

    assert dispatcher.dropped() == set()
    assert dispatcher.remaining("a") == []
    assert ids(dispatcher.remaining("b")) == ["s-r0-a-000", "s-r0-a-001"]


def test_load_for_a_dropped_drone_goes_to_the_survivors() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": [], "b": []})
    fleet.kill("a")
    fleet.clock.advance(DROPOUT_S)
    dispatcher.tick()

    dispatcher.load({"a": route("a", 1, round_index=1)})
    dispatcher.tick()

    assert fleet.sent == [("b", "s-r1-a-000")]


@pytest.mark.parametrize("seed", range(20))
def test_random_dropouts_finish_every_mission_exactly_once(seed: int) -> None:
    rng = random.Random(seed)
    drone_ids = ["a", "b", "c", "d"]
    fleet = FakeFleet(*drone_ids)
    dispatcher = make_dispatcher(fleet)
    routes = {drone_id: route(drone_id, 8) for drone_id in drone_ids}
    dispatcher.load(routes)
    # Down drones and the time they come back: some outages are shorter
    # than the dropout limit, some longer, some last past the survey.
    down_until: dict[str, float] = {}
    fleet_start = fleet.clock()
    ever_dropped: set[str] = set()

    for _ in range(2000):
        if dispatcher.is_done():
            break
        fleet.clock.advance(1.0)
        now = fleet.clock()
        for drone_id, back_at in list(down_until.items()):
            if now >= back_at:
                fleet.revive(drone_id)
                del down_until[drone_id]
        up = [drone_id for drone_id in drone_ids if drone_id not in down_until]
        if now == fleet_start + 3.0:
            # Every seed drops at least one drone mid-route.
            fleet.kill(up[0])
            down_until[up[0]] = now + 1e9
        elif up and rng.random() < 0.15:
            victim = rng.choice(up)
            fleet.kill(victim)
            down_until[victim] = now + rng.choice([2.0, 5.0, 15.0, 40.0, 1e9])
        if len(down_until) == len(drone_ids):
            # Someone must come back, or the pool is never flown.
            back = rng.choice(drone_ids)
            down_until[back] = min(down_until[back], now + 20.0)
        dispatcher.tick()
        ever_dropped |= dispatcher.dropped()
        for drone_id in drone_ids:
            mission = dispatcher.in_flight(drone_id)
            # Some legs take longer than one tick.
            if drone_id not in down_until and mission is not None and rng.random() < 0.3:
                fleet.fly(drone_id, mission.task_id)

    assert dispatcher.is_done()
    assert ever_dropped, "seed never dropped a drone; the test proves nothing"
    finished = Counter(task for _, task in dispatcher.completed())
    planned = {m.task_id for planned in routes.values() for m in planned}
    assert set(finished) == planned
    assert set(finished.values()) == {1}


@pytest.mark.parametrize("dropout_s", [0.0, -1.0])
def test_non_positive_dropout_is_refused(dropout_s: float) -> None:
    with pytest.raises(ValueError, match="dropout_after_s"):
        RouteDispatcher(FakeFleet(), FakeClock(), dropout_after_s=dropout_s)


class StampingFleet(FakeFleet):
    """
    Stamps every drone on every read, even when disconnected or silent,
    like a manager that keeps timestamping a closed link.
    """

    def __init__(self, *drone_ids: str, stamp_offset_s: float = 0.0) -> None:
        super().__init__(*drone_ids)
        self.stamp_offset_s = stamp_offset_s

    def get_all_drones(self) -> dict[str, DroneState]:
        for state in self.states.values():
            state.last_update_timestamp = self.clock() + self.stamp_offset_s
        return super().get_all_drones()


def test_disconnected_drone_is_dropped_even_if_still_stamped() -> None:
    fleet = StampingFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2), "b": []})
    dispatcher.tick()
    fleet.states["a"].connected = False

    for _ in range(int(DROPOUT_S)):
        fleet.clock.advance(1.0)
        dispatcher.tick()

    assert dispatcher.dropped() == {"a"}
    assert ids(dispatcher.remaining("b")) == ["s-r0-a-000", "s-r0-a-001"]


def test_timestamp_in_the_future_does_not_keep_a_dead_drone_alive() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1), "b": []})
    fleet.silent.add("a")
    fleet.states["a"].last_update_timestamp = fleet.clock() + 1000.0

    # The first read counts as a report, so the drop is one tick later.
    for _ in range(int(DROPOUT_S) + 1):
        fleet.clock.advance(1.0)
        dispatcher.tick()

    assert dispatcher.dropped() == {"a"}


def test_manager_clock_ahead_of_ours_is_still_reporting() -> None:
    fleet = StampingFleet("a", stamp_offset_s=1000.0)
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 1)})

    for _ in range(3 * int(DROPOUT_S)):
        fleet.clock.advance(1.0)
        dispatcher.tick()

    assert dispatcher.dropped() == set()


def test_task_dropped_while_arrived_is_redone_once() -> None:
    fleet = FakeFleet("a", "b")
    dispatcher = make_dispatcher(fleet)
    dispatcher.load({"a": route("a", 2), "b": route("b", 1)})
    dispatcher.tick()
    # a reaches 000 but dies before the dispatcher sees the report.
    fleet.fly("a", "s-r0-a-000")
    fleet.kill("a")
    fleet.clock.advance(DROPOUT_S)
    dispatcher.tick()
    assert ids(dispatcher.remaining("b")) == [
        "s-r0-b-000",
        "s-r0-a-000",
        "s-r0-a-001",
    ]

    # Back online, still reporting ARRIVED on the task it lost.
    fleet.revive("a")
    dispatcher.tick()

    assert dispatcher.completed() == []
    assert dispatcher.remaining("a") == []
