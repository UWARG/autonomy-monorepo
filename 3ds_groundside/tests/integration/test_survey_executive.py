"""
Whole surveys flown by real MockDrones over websockets: a nominal run, a run
where a drone is killed partway, and a run where the stitcher rejects photos.
All three must end COMPLETE with the area fully covered.
"""

from __future__ import annotations

import re
import threading
from collections import defaultdict
from collections.abc import Callable, Iterator

import pytest

from src.image_stitcher.message import ImageMessage
from src.survey_executive import (
    DroneActivity,
    SurveyExecutive,
    SurveyProgress,
    SurveyRunner,
    SurveyState,
)
from tests.integration.survey_harness import FleetHarness, flying
from tests.unit.survey_executive.factories import make_request

DRONES = ("drone-01", "drone-02", "drone-03")
# Short enough for CI, with room for a loaded runner: drones report every
# 20 ms, so a live drone never goes a full dropout window without a report.
SETTLE_S = 0.3
ACK_TIMEOUT_S = 0.5
DROPOUT_AFTER_S = 1.0
TICK_HZ = 50.0
SURVEY_TIMEOUT_S = 30.0

# survey-1-r<round>-<planned drone>-<index>
TASK_ID = re.compile(r"^survey-1-r(\d+)-(drone-\d+)-(\d{3})$")


class StubStitcher:
    """
    Takes every photo and rejects the ones ``reject`` picks by image ID,
    recording them the way ``ImageStitcher.rejections`` does.
    """

    def __init__(self, reject: Callable[[str], bool] = lambda image_id: False) -> None:
        self._reject = reject
        self._lock = threading.Lock()
        self._rejections: list[tuple[str, str, str]] = []

    @property
    def rejections(self) -> list[tuple[str, str, str]]:
        with self._lock:
            return list(self._rejections)

    def add(self, photo: ImageMessage) -> None:
        if self._reject(photo.image_id):
            with self._lock:
                self._rejections.append(
                    (photo.drone_id, photo.image_id, "too few feature matches")
                )


@pytest.fixture
def fleet() -> Iterator[FleetHarness]:
    with flying(DRONES) as harness:
        yield harness


def make_executive(
    fleet: FleetHarness, stitcher: StubStitcher | None = None
) -> SurveyExecutive:
    """
    An executive wired to the fleet the way groundside will wire it: every
    photo goes to the stitcher and to the executive.
    """

    stitcher = stitcher or StubStitcher()
    executive = SurveyExecutive(
        make_request(),
        fleet.manager,
        stitcher_rejections=lambda: stitcher.rejections,
        settle_s=SETTLE_S,
        ack_timeout_s=ACK_TIMEOUT_S,
        dropout_after_s=DROPOUT_AFTER_S,
    )

    def deliver(photo: ImageMessage) -> None:
        stitcher.add(photo)
        executive.on_photo(photo)

    fleet.photo_sink = deliver
    return executive


def fly_survey(
    fleet: FleetHarness,
    executive: SurveyExecutive,
    partway: tuple[str, int] | None = None,
) -> SurveyProgress:
    """
    Starts the survey and waits for it to end. ``partway`` = (drone, count)
    kills that drone once the executive has counted that many of its
    missions as done.
    """

    executive.start()
    runner = SurveyRunner(executive, rate_hz=TICK_HZ)
    runner.start()
    try:
        if partway is not None:
            victim, count = partway
            # Wait on the executive's count, not the photos: a photo comes
            # with the first ARRIVED report, and a drone killed before the
            # next tick still has that mission in flight, so it is reflown.
            fleet.wait_for(
                lambda: missions_done_by(executive, victim) >= count,
                timeout_s=SURVEY_TIMEOUT_S,
                what=f"{victim} to finish {count} missions",
            )
            fleet.kill(victim)
        fleet.wait_for(
            lambda: executive.state.is_final,
            timeout_s=SURVEY_TIMEOUT_S,
            what="the survey to end",
        )
    finally:
        executive.abort("test over")  # no effect once final
        runner.stop()
    return executive.progress()


def missions_done_by(executive: SurveyExecutive, drone_id: str) -> int:
    drones = {drone.drone_id: drone for drone in executive.progress().drones}
    return drones[drone_id].missions_done


def photos_by_route(fleet: FleetHarness, marker: str) -> int:
    return sum(marker in photo.image_id for photo in fleet.photos)


def flown_in_order(fleet: FleetHarness) -> dict[str, list[str]]:
    """
    The tasks each drone accepted, first acceptance only (resends re-ack).
    """

    flown: dict[str, list[str]] = defaultdict(list)
    for drone_id, task_id, accepted in fleet.manager.acks:
        if accepted and task_id not in flown[drone_id]:
            flown[drone_id].append(task_id)
    return dict(flown)


def assert_routes_flown_in_order(fleet: FleetHarness) -> None:
    """
    Within each planned route (one round, one planned drone), every drone
    flew its share in the planned order.
    """

    for drone_id, task_ids in flown_in_order(fleet).items():
        indices: dict[tuple[str, str], list[int]] = defaultdict(list)
        for task_id in task_ids:
            match = TASK_ID.match(task_id)
            assert match is not None, task_id
            round_index, planned_for, index = match.groups()
            indices[(round_index, planned_for)].append(int(index))
        for route, flown in indices.items():
            assert flown == sorted(flown), f"{drone_id} flew {route} out of order"


def assert_fully_covered(progress: SurveyProgress) -> None:
    assert progress.state is SurveyState.COMPLETE, progress.abort_reason
    assert progress.fully_covered
    assert progress.gaps_remaining == 0
    assert progress.percent_covered == pytest.approx(100.0)
    assert progress.missions_done == progress.missions_total
    assert progress.missions_unassigned == 0


def test_nominal_survey_covers_the_area(fleet: FleetHarness) -> None:
    executive = make_executive(fleet)

    progress = fly_survey(fleet, executive)

    assert_fully_covered(progress)
    assert progress.round_index == 0  # nothing to reshoot
    assert progress.photos_rejected == 0
    assert progress.photos_received == progress.missions_total
    flown = flown_in_order(fleet)
    assert set(flown) == set(DRONES)
    # Every drone flew exactly its own planned route.
    for drone_id, task_ids in flown.items():
        assert all(f"-{drone_id}-" in task_id for task_id in task_ids)
    assert_routes_flown_in_order(fleet)


def test_survey_survives_a_drone_killed_partway(fleet: FleetHarness) -> None:
    executive = make_executive(fleet)

    progress = fly_survey(fleet, executive, partway=("drone-02", 3))

    assert_fully_covered(progress)
    drones = {drone.drone_id: drone for drone in progress.drones}
    assert drones["drone-02"].activity is DroneActivity.DROPPED
    assert drones["drone-02"].missions_remaining == 0
    flown = flown_in_order(fleet)
    taken_over = [
        task_id
        for drone_id in ("drone-01", "drone-03")
        for task_id in flown[drone_id]
        if "-r0-drone-02-" in task_id
    ]
    assert taken_over, "nobody took over drone-02's route"
    assert len(taken_over) == len(set(taken_over))
    # Only what the executive counted done is drone-02's. A mission it was
    # flying when killed goes back to the pool, even if its photo got out.
    # drone-02 flies its own route in order, so those are its first ones.
    done_by_02 = set(flown["drone-02"][: drones["drone-02"].missions_done])
    assert done_by_02.isdisjoint(taken_over)
    route_02 = set(flown["drone-02"]) | set(taken_over)
    photographed = {
        photo.image_id for photo in fleet.photos if "-r0-drone-02-" in photo.image_id
    }
    assert photographed == route_02, "a drone-02 mission was never photographed"
    assert_routes_flown_in_order(fleet)


def test_survey_reshoots_photos_the_stitcher_rejected(fleet: FleetHarness) -> None:
    # All of drone-01's first-round photos: a whole strip, so overlap with
    # the neighbouring routes can't hide the gap.
    stitcher = StubStitcher(reject=lambda image_id: "-r0-drone-01-" in image_id)
    executive = make_executive(fleet, stitcher)

    progress = fly_survey(fleet, executive)

    assert_fully_covered(progress)
    rejected = {image_id for _, image_id, _ in stitcher.rejections}
    assert rejected <= set(flown_in_order(fleet)["drone-01"])
    assert len(rejected) == photos_by_route(fleet, "-r0-drone-01-")
    assert progress.photos_rejected == len(rejected)
    assert progress.round_index >= 1
    reshoots = [
        task_id
        for task_ids in flown_in_order(fleet).values()
        for task_id in task_ids
        if "-r0-" not in task_id
    ]
    assert reshoots, "no reshoot missions were flown"
    assert_routes_flown_in_order(fleet)
