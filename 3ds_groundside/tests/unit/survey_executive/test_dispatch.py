"""
Unit tests for route-preserving dispatch of planned Missions.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState, MissionState
from src.survey_executive.dispatch import DroneManagerLike, RouteDispatcher
from tests.fakes import FakeClock
from tests.unit.survey_executive.factories import LAT0, LON0, make_drone

TIMEOUT_S = 3.0


class FakeFleet:
    """
    Drone states the test sets by hand, plus a log of every command sent.

    ``send_location_command_sync`` returns ``connected``, like the planned
    #153 method: sent or not, never whether the drone took it.
    """

    def __init__(self, *drone_ids: str) -> None:
        self.states = {drone_id: make_drone(drone_id) for drone_id in drone_ids}
        self.sent: list[tuple[str, str]] = []

    def get_all_drones(self) -> dict[str, DroneState]:
        return {drone_id: replace(state) for drone_id, state in self.states.items()}

    def send_location_command_sync(
        self, drone_id: str, task_id: str, target: Coordinate, yaw: float
    ) -> bool:
        state = self.states.get(drone_id)
        if state is None or not state.connected:
            return False
        self.sent.append((drone_id, task_id))
        return True

    def report(
        self, drone_id: str, mission_state: MissionState, task_id: str | None
    ) -> None:
        self.states[drone_id].mission_state = mission_state
        self.states[drone_id].active_task_id = task_id

    def fly(self, drone_id: str, task_id: str) -> None:
        """
        The drone takes the task and reaches it before the next tick.
        """

        self.report(drone_id, MissionState.ARRIVED, task_id)


def route(drone_id: str, count: int, round_index: int = 0) -> list[Mission]:
    return [
        Mission(
            task_id=f"s-r{round_index}-{drone_id}-{index:03d}",
            target=Coordinate(LAT0 + index * 1e-5, LON0, 30.0),
            yaw=0.5,
        )
        for index in range(count)
    ]


def make_dispatcher(
    fleet: FakeFleet, clock: FakeClock | None = None
) -> RouteDispatcher:
    return RouteDispatcher(fleet, clock or FakeClock(), ack_timeout_s=TIMEOUT_S)


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
    clock = FakeClock()
    dispatcher = make_dispatcher(fleet, clock)
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
    clock = FakeClock()
    dispatcher = make_dispatcher(fleet, clock)
    dispatcher.load({"a": route("a", 1)})
    dispatcher.tick()
    fleet.report("a", MissionState.MOVING, "s-r0-a-000")
    dispatcher.tick()

    clock.advance(10 * TIMEOUT_S)
    dispatcher.tick()

    assert len(fleet.sent) == 1


def test_resends_when_a_drone_loses_a_task_it_had_taken() -> None:
    fleet = FakeFleet("a")
    clock = FakeClock()
    dispatcher = make_dispatcher(fleet, clock)
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
    clock = FakeClock()
    dispatcher = make_dispatcher(fleet, clock)
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
