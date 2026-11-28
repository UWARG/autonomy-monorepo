"""
Unit tests for the survey executive's state machine, reshoot loop and
progress, against the fake fleet with synthetic nadir photos.
"""

from __future__ import annotations

import json
import math
import threading
from collections.abc import Callable

import numpy as np
import pytest
from utils.src.types import Coordinate

from src.groundside_manager.groundside_manager import DroneState, MissionState
from src.image_stitcher.message import ImageMessage
from src.survey_executive import (
    DroneActivity,
    PlanningError,
    SurveyExecutive,
    SurveyRunner,
    SurveyState,
)
from tests.unit.survey_executive.factories import RECTANGLE, make_request
from tests.unit.survey_executive.fakes import FakeFleet

SETTLE_S = 3.0
DRONES = ("drone-01", "drone-02")


def photo_at(
    drone_id: str, task_id: str, target: Coordinate, yaw_rad: float, **overrides: float
) -> ImageMessage:
    """
    A level photo taken at the setpoint, facing the commanded yaw.
    """

    fields = {
        "lat": target.lat,
        "lon": target.lon,
        "alt": target.alt,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": math.degrees(yaw_rad),
    }
    fields.update(overrides)
    return ImageMessage(
        img=np.zeros((2, 2, 3), dtype=np.uint8),
        drone_id=drone_id,
        image_id=task_id,
        **fields,
    )


class Harness:
    """
    Flies the fake fleet: every tick each live drone reaches its last
    command and sends a photo there, unless ``keep_photo`` says no.
    """

    def __init__(self, **request_overrides: object) -> None:
        self.fleet = FakeFleet(*DRONES)
        self.rejected: list[tuple[str, str, str]] = []
        self.keep_photo: Callable[[str], bool] = lambda task_id: True
        # Photo image_id from (drone_id, task_id); the task ID by default.
        self.name_photo: Callable[[str, str], str] = lambda drone_id, task_id: task_id
        self.held: list[ImageMessage] = []
        self.executive = SurveyExecutive(
            make_request(**request_overrides),
            self.fleet,
            stitcher_rejections=lambda: list(self.rejected),
            clock=self.fleet.clock,
            settle_s=SETTLE_S,
        )

    def step(self) -> None:
        self.fleet.clock.advance(1.0)
        self.executive.tick()
        for drone_id, (task_id, target, yaw) in self.fleet.commands.items():
            state = self.fleet.states[drone_id]
            if drone_id in self.fleet.silent or state.active_task_id == task_id:
                continue
            state.position = target
            self.fleet.fly(drone_id, task_id)
            photo = photo_at(drone_id, self.name_photo(drone_id, task_id), target, yaw)
            if self.keep_photo(task_id):
                self.executive.on_photo(photo)
            else:
                self.held.append(photo)

    def run(self, max_steps: int = 400) -> None:
        for _ in range(max_steps):
            if self.executive.state.is_final:
                return
            self.step()

    def run_until(self, done: Callable[[], bool], max_steps: int = 400) -> None:
        for _ in range(max_steps):
            if done():
                return
            self.step()
        raise AssertionError("condition never reached")

    def sent_ids(self) -> list[str]:
        return [task_id for _, task_id in self.fleet.sent]


def drop_first_photos(harness: Harness, count: int) -> None:
    """
    Loses drone-01's first ``count`` round-0 photos: a strip of the area.
    """

    lost = {f"survey-1-r0-drone-01-{index:03d}" for index in range(count)}
    harness.keep_photo = lambda task_id: task_id not in lost


def round_flown(harness: Harness) -> bool:
    progress = harness.executive.progress()
    return progress.missions_done == progress.missions_total


def test_nominal_survey_completes_fully_covered_in_one_round() -> None:
    harness = Harness()
    harness.executive.start()
    assert harness.executive.state is SurveyState.SURVEYING

    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert progress.fully_covered
    assert progress.percent_covered == pytest.approx(100.0)
    assert progress.round_index == 0
    assert progress.gaps_remaining == 0
    assert progress.missions_done == progress.missions_total > 0
    assert progress.photos_received == progress.missions_total
    assert not any("-r1-" in task_id for task_id in harness.sent_ids())


def test_lost_photos_are_reshot_then_complete() -> None:
    harness = Harness()
    drop_first_photos(harness, 4)
    harness.executive.start()

    harness.run_until(lambda: harness.executive.state is SurveyState.RESHOOTING)
    assert harness.executive.progress().gaps_remaining > 0
    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert progress.fully_covered
    assert progress.round_index == 1
    assert any("-r1-" in task_id for task_id in harness.sent_ids())


def test_rejected_photos_do_not_count_and_are_reshot() -> None:
    harness = Harness()
    # The same strip drop_first_photos loses, so the rejections leave a gap.
    rejected = {f"survey-1-r0-drone-01-{index:03d}" for index in range(4)}

    def reject(task_id: str) -> bool:
        if task_id in rejected:
            harness.rejected.append(("drone-01", task_id, "too few matches"))
        return True  # the photo arrives, then the stitcher rejects it

    harness.keep_photo = reject
    harness.executive.start()
    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert progress.fully_covered
    assert progress.round_index == 1
    assert progress.photos_rejected == len(rejected)


def test_gaps_left_after_the_last_round_still_complete() -> None:
    harness = Harness(max_reshoot_rounds=0)
    drop_first_photos(harness, 4)
    harness.executive.start()

    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert not progress.fully_covered
    assert progress.gaps_remaining > 0
    assert progress.percent_covered < 100.0
    assert not any("-r1-" in task_id for task_id in harness.sent_ids())


def test_gaps_are_not_judged_until_the_round_settles() -> None:
    harness = Harness()
    drop_first_photos(harness, 4)
    harness.executive.start()
    harness.run_until(lambda: round_flown(harness))

    # The held photos turn up late, but within the settle time.
    harness.fleet.clock.advance(SETTLE_S - 1.5)
    harness.executive.tick()
    assert harness.executive.state is SurveyState.SURVEYING
    for photo in harness.held:
        harness.executive.on_photo(photo)
    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert progress.fully_covered
    assert progress.round_index == 0


def test_dropped_drone_still_ends_fully_covered() -> None:
    harness = Harness()
    harness.executive.start()
    harness.step()
    harness.step()
    harness.fleet.kill("drone-02")

    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert progress.fully_covered
    assert sum(drone.missions_done for drone in progress.drones) == progress.missions_done
    by_id = {drone.drone_id: drone for drone in progress.drones}
    assert by_id["drone-02"].activity is DroneActivity.DROPPED
    assert by_id["drone-02"].missions_remaining == 0


def test_waits_for_a_drone_when_none_is_left_to_reshoot() -> None:
    harness = Harness()
    drop_first_photos(harness, 4)
    harness.executive.start()
    harness.run_until(lambda: round_flown(harness))
    for drone_id in DRONES:
        harness.fleet.kill(drone_id)

    for _ in range(30):
        harness.step()
    assert harness.executive.state is SurveyState.SURVEYING

    harness.fleet.revive("drone-01")
    harness.run()
    assert harness.executive.state is SurveyState.COMPLETE
    assert harness.executive.progress().fully_covered


def test_abort_while_surveying_stops_all_commands() -> None:
    harness = Harness()
    harness.executive.start()
    harness.step()

    harness.executive.abort()
    sent = len(harness.fleet.sent)
    for _ in range(10):
        harness.step()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.ABORTED
    assert progress.abort_reason == "operator abort"
    assert len(harness.fleet.sent) == sent


def test_abort_while_reshooting() -> None:
    harness = Harness()
    drop_first_photos(harness, 4)
    harness.executive.start()
    harness.run_until(lambda: harness.executive.state is SurveyState.RESHOOTING)

    harness.executive.abort("wind")
    sent = len(harness.fleet.sent)
    harness.step()

    assert harness.executive.state is SurveyState.ABORTED
    assert harness.executive.progress().abort_reason == "wind"
    assert len(harness.fleet.sent) == sent


def test_abort_after_complete_changes_nothing() -> None:
    harness = Harness()
    harness.executive.start()
    harness.run()

    harness.executive.abort()

    assert harness.executive.state is SurveyState.COMPLETE
    assert harness.executive.progress().abort_reason is None


def test_abort_before_start_prevents_starting() -> None:
    harness = Harness()
    harness.executive.abort()

    with pytest.raises(RuntimeError, match="ABORTED"):
        harness.executive.start()
    assert harness.fleet.sent == []


def test_invalid_request_is_refused_before_any_command() -> None:
    harness = Harness(boundary=RECTANGLE[:2])

    with pytest.raises(ValueError, match="at least 3 points"):
        harness.executive.start()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.ABORTED
    assert progress.abort_reason is not None
    assert progress.abort_reason.startswith("refused to start")
    harness.step()
    assert harness.fleet.sent == []


def test_no_connected_drones_is_refused() -> None:
    harness = Harness()
    for drone_id in DRONES:
        harness.fleet.states[drone_id].connected = False

    with pytest.raises(ValueError, match="at least one drone"):
        harness.executive.start()
    assert harness.executive.state is SurveyState.ABORTED


def test_only_connected_drones_are_planned() -> None:
    harness = Harness()
    harness.fleet.states["drone-02"].connected = False

    harness.executive.start()

    assert [d.drone_id for d in harness.executive.progress().drones] == ["drone-01"]


def test_start_twice_is_refused() -> None:
    harness = Harness()
    harness.executive.start()

    with pytest.raises(RuntimeError, match="SURVEYING"):
        harness.executive.start()


def test_photo_that_cannot_be_placed_counts_as_received_only() -> None:
    harness = Harness()
    harness.executive.start()
    target = Coordinate(RECTANGLE[0].lat, RECTANGLE[0].lon, 30.0)

    harness.executive.on_photo(
        photo_at("drone-01", "x", target, 0.0, roll_deg=float("nan"))
    )
    harness.executive.tick()

    progress = harness.executive.progress()
    assert progress.photos_received == 1
    assert progress.percent_covered == 0.0


def test_photos_after_the_end_are_ignored() -> None:
    harness = Harness()
    harness.executive.start()
    harness.run()
    received = harness.executive.progress().photos_received
    target = harness.fleet.commands["drone-01"][1]

    harness.executive.on_photo(photo_at("drone-01", "late", target, 0.0))

    # Nothing ticks after the end, so a late photo must not even be queued.
    assert harness.executive._inbox == []
    harness.executive.tick()
    assert harness.executive.progress().photos_received == received


def test_photos_without_image_id_never_count_as_coverage() -> None:
    harness = Harness(max_reshoot_rounds=0)
    # The stitcher would record a rejection of these under "", which can't
    # be told apart, so they must not be trusted as coverage.
    harness.name_photo = lambda drone_id, task_id: ""
    harness.executive.start()
    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert not progress.fully_covered
    assert progress.percent_covered == 0.0
    assert progress.photos_received == progress.missions_total


def test_rejection_arriving_on_the_settle_deadline_tick_still_counts() -> None:
    harness = Harness()
    harness.executive.start()
    harness.run_until(
        lambda: (progress := harness.executive.progress()).missions_done
        == progress.missions_total
    )
    deadline = harness.fleet.clock() + SETTLE_S
    rejected = [f"survey-1-r0-drone-01-{index:03d}" for index in range(4)]
    read_drones = harness.fleet.get_all_drones

    def rejections_arrive_during_the_poll() -> dict[str, DroneState]:
        # The stitcher rejects them while the deadline tick polls the fleet,
        # after the tick has already read the rejections once.
        if harness.fleet.clock() >= deadline and not harness.rejected:
            harness.rejected.extend(("drone-01", t, "blurry") for t in rejected)
        return read_drones()

    harness.fleet.get_all_drones = rejections_arrive_during_the_poll  # type: ignore[method-assign]
    harness.run()

    progress = harness.executive.progress()
    assert progress.photos_rejected == len(rejected)
    assert progress.round_index == 1  # the gap was reshot, not missed
    assert progress.fully_covered


def test_photo_with_a_null_attitude_is_skipped_not_fatal() -> None:
    harness = Harness()
    harness.executive.start()
    target = Coordinate(RECTANGLE[0].lat, RECTANGLE[0].lon, 30.0)

    harness.executive.on_photo(photo_at("drone-01", "x", target, 0.0, roll_deg=None))
    harness.executive.tick()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.SURVEYING
    assert progress.photos_received == 1
    assert progress.percent_covered == 0.0


def test_planner_refusal_at_start_aborts_before_any_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def refuse(*args: object, **kwargs: object) -> None:
        raise PlanningError("planner rejected the area or fleet")

    monkeypatch.setattr("src.survey_executive.executive.plan_routes", refuse)
    harness = Harness()

    with pytest.raises(PlanningError):
        harness.executive.start()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.ABORTED
    assert progress.abort_reason is not None
    assert progress.abort_reason.startswith("refused to start")
    harness.step()
    assert harness.fleet.sent == []


def test_progress_is_json_ready_with_per_drone_status() -> None:
    harness = Harness()
    harness.executive.start()
    harness.step()

    data = json.loads(json.dumps(harness.executive.progress().to_dict()))

    assert data["state"] == "SURVEYING"
    assert data["survey_id"] == "survey-1"
    assert {drone["drone_id"] for drone in data["drones"]} == set(DRONES)
    assert {drone["activity"] for drone in data["drones"]} <= {"FLYING", "IDLE"}
    assert all(drone["in_flight_task_id"] for drone in data["drones"])


def test_progress_before_start() -> None:
    progress = Harness().executive.progress()

    assert progress.state is SurveyState.PLANNING
    assert progress.percent_covered == 0.0
    assert progress.drones == []


def test_photos_from_other_threads_are_all_counted() -> None:
    harness = Harness()
    harness.executive.start()
    target = Coordinate(RECTANGLE[0].lat, RECTANGLE[0].lon, 30.0)

    def send(worker: int) -> None:
        for index in range(50):
            harness.executive.on_photo(
                photo_at("drone-01", f"w{worker}-{index}", target, 0.0)
            )

    workers = [threading.Thread(target=send, args=(n,)) for n in range(4)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()
    harness.executive.tick()

    assert harness.executive.progress().photos_received == 200


def test_negative_settle_time_is_refused() -> None:
    with pytest.raises(ValueError, match="settle_s"):
        SurveyExecutive(make_request(), FakeFleet(), settle_s=-1.0)


def test_runner_ticks_until_aborted() -> None:
    harness = Harness()
    harness.executive.start()
    runner = SurveyRunner(harness.executive, rate_hz=200.0)

    runner.start()
    harness.executive.abort()
    runner.stop()

    assert harness.executive.state is SurveyState.ABORTED


def test_runner_aborts_the_survey_if_a_tick_fails() -> None:
    class BrokenFleet(FakeFleet):
        broken = False

        def get_all_drones(self) -> dict[str, DroneState]:
            if self.broken:
                raise ConnectionError("manager gone")
            return super().get_all_drones()

    fleet = BrokenFleet(*DRONES)
    executive = SurveyExecutive(make_request(), fleet, clock=fleet.clock)
    executive.start()
    fleet.broken = True
    runner = SurveyRunner(executive, rate_hz=200.0)

    runner.start()
    for _ in range(200):
        if executive.state.is_final:
            break
        threading.Event().wait(0.01)
    runner.stop()

    progress = executive.progress()
    assert progress.state is SurveyState.ABORTED
    assert progress.abort_reason == "internal error: manager gone"


def test_drones_report_arrived_on_their_last_task() -> None:
    # Guards the harness itself: the fleet flies exactly what it was sent.
    harness = Harness()
    harness.executive.start()
    harness.step()

    for drone_id, (task_id, _, _) in harness.fleet.commands.items():
        state = harness.fleet.states[drone_id]
        assert state.mission_state is MissionState.ARRIVED
        assert state.active_task_id == task_id


def camera_file_names(harness: Harness) -> None:
    """
    Names photos like a camera does: a counter per drone, so both drones
    produce IMG_0000, IMG_0001, ...
    """

    counters: dict[str, int] = {}

    def name(drone_id: str, task_id: str) -> str:
        counters[drone_id] = counters.get(drone_id, 0) + 1
        return f"IMG_{counters[drone_id] - 1:04d}"

    harness.name_photo = name


def test_same_file_names_from_two_drones_do_not_overwrite() -> None:
    harness = Harness()
    camera_file_names(harness)
    harness.executive.start()

    harness.run()

    progress = harness.executive.progress()
    assert progress.fully_covered
    assert progress.round_index == 0


def test_a_rejection_only_excludes_that_drones_photo() -> None:
    harness = Harness()
    camera_file_names(harness)
    # drone-02's first photo is rejected; drone-01's IMG_0000 is fine.
    harness.rejected.append(("drone-02", "IMG_0000", "too few matches"))
    harness.executive.start()

    harness.run()

    progress = harness.executive.progress()
    assert progress.fully_covered
    assert progress.round_index <= 1
    assert progress.photos_rejected == 1


def test_rejection_of_a_photo_never_received_is_not_counted() -> None:
    harness = Harness()
    harness.rejected.append(("drone-09", "elsewhere", "too few matches"))
    harness.executive.start()
    harness.executive.tick()

    assert harness.executive.progress().photos_rejected == 0


@pytest.mark.parametrize(
    ("lat", "lon"), [(math.inf, -80.5), (43.47, math.nan), (95.0, -80.5), (43.47, 400.0)]
)
def test_photo_with_bad_gps_is_skipped_not_fatal(lat: float, lon: float) -> None:
    harness = Harness()
    harness.executive.start()

    harness.executive.on_photo(photo_at("drone-01", "x", Coordinate(lat, lon, 30.0), 0.0))
    harness.executive.tick()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.SURVEYING
    assert progress.photos_received == 1
    assert progress.percent_covered == 0.0


@pytest.mark.parametrize("max_rounds", [0, 1, 2])
def test_reshoot_rounds_stop_exactly_at_the_limit(max_rounds: int) -> None:
    harness = Harness(max_reshoot_rounds=max_rounds)
    harness.keep_photo = lambda task_id: False  # nothing ever arrives
    harness.executive.start()

    harness.run(max_steps=1000)

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert not progress.fully_covered
    assert progress.round_index == max_rounds
    flown = {int(task_id.split("-r")[1].split("-")[0]) for task_id in harness.sent_ids()}
    assert flown == set(range(max_rounds + 1))


def test_reshoot_the_planner_refuses_does_not_count_as_a_round(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import src.survey_executive.executive as executive_module

    def refuse(request, gaps, anchor, drones, round_index):  # type: ignore[no-untyped-def]
        return {drone.drone_id: [] for drone in drones}, list(gaps)

    monkeypatch.setattr(executive_module, "plan_reshoot", refuse)
    harness = Harness()
    drop_first_photos(harness, 4)
    harness.executive.start()

    harness.run()

    progress = harness.executive.progress()
    assert progress.state is SurveyState.COMPLETE
    assert not progress.fully_covered
    assert progress.round_index == 0
    assert progress.gaps_unplanned == progress.gaps_remaining > 0


def test_runner_refuses_a_survey_that_was_never_started() -> None:
    runner = SurveyRunner(Harness().executive)

    with pytest.raises(RuntimeError, match="start the survey"):
        runner.start()


def test_photos_sent_before_start_are_dropped() -> None:
    harness = Harness()
    target = Coordinate(RECTANGLE[0].lat, RECTANGLE[0].lon, 30.0)
    harness.executive.on_photo(photo_at("drone-01", "early", target, 0.0))

    harness.executive.start()
    harness.executive.tick()

    progress = harness.executive.progress()
    assert progress.photos_received == 0
    assert progress.percent_covered == 0.0


def test_a_photo_sent_twice_counts_once() -> None:
    harness = Harness()
    harness.executive.start()
    target = Coordinate(RECTANGLE[0].lat, RECTANGLE[0].lon, 30.0)

    for _ in range(2):
        harness.executive.on_photo(photo_at("drone-01", "IMG_0001", target, 0.0))
    harness.executive.tick()

    assert harness.executive.progress().photos_received == 1


def test_photos_without_an_id_each_count() -> None:
    harness = Harness()
    harness.executive.start()
    target = Coordinate(RECTANGLE[0].lat, RECTANGLE[0].lon, 30.0)

    for _ in range(2):
        harness.executive.on_photo(photo_at("drone-01", "", target, 0.0))
    harness.executive.tick()

    assert harness.executive.progress().photos_received == 2
