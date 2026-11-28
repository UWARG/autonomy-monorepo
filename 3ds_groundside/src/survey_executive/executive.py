"""
Runs one survey from a boundary polygon to a covered map.

PLANNING -> SURVEYING -> (RESHOOTING ->)* COMPLETE, or ABORTED from any
state that is not final.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable, Iterable

from shapely.geometry import Polygon
from utils.src.coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
)
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState
from src.image_stitcher.message import ImageMessage

from .coverage import CoverageMap
from .dispatch import (
    DEFAULT_ACK_TIMEOUT_S,
    DEFAULT_DROPOUT_AFTER_S,
    DroneManagerLike,
    RouteDispatcher,
)
from .footprint import photo_footprint
from .planning import PlanningError, plan_routes
from .progress import DroneActivity, DroneProgress, SurveyProgress, SurveyState
from .request import SurveyRequest, validate_request
from .reshoot import plan_reshoot

logger = logging.getLogger(__name__)

# Photos and stitcher rejections arrive after ARRIVED, and the stitcher holds
# its first photos back, so wait this long before calling a spot a gap.
DEFAULT_SETTLE_S = 5.0

# (drone_id, image_id, reason), the shape of ``ImageStitcher.rejections``.
Rejection = tuple[str, str, str]


def _no_rejections() -> list[Rejection]:
    return []


def _photo_key(drone_id: str, image_id: str) -> str:
    return f"{drone_id}/{image_id}"


class SurveyExecutive:
    """
    Plans the survey, flies each drone's route in order, tracks coverage from
    the photos, and reshoots gaps until none is left or the rounds run out.

    ``stitcher_rejections`` returns every photo the stitcher has rejected so
    far (e.g. ``lambda: stitcher.rejections``); those photos never count as
    coverage. Feeding photos to the stitcher is the caller's job.

    ``tick`` does all the work and is meant to run on one thread (see
    ``SurveyRunner``). ``on_photo`` may be called from any thread; the other
    methods are safe to call from any thread too, but wait for a running
    tick, which includes the manager's command sends. A manager whose sync
    send blocks for long would also delay ``progress`` and ``abort``.

    If gaps remain after ``max_reshoot_rounds``, the survey still ends
    COMPLETE, with ``fully_covered`` False and the gaps counted in progress.
    If no drone is left, it waits until one rejoins or the operator aborts.

    Photos are matched to stitcher rejections by (drone_id, image_id), the
    way the stitcher names them, so a photo with no ``image_id`` never
    counts as coverage. Photos sent before ``start`` or after the end are
    dropped.
    """

    def __init__(
        self,
        request: SurveyRequest,
        manager: DroneManagerLike,
        stitcher_rejections: Callable[[], Iterable[Rejection]] = _no_rejections,
        clock: Callable[[], float] = time.monotonic,
        settle_s: float = DEFAULT_SETTLE_S,
        ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S,
        dropout_after_s: float = DEFAULT_DROPOUT_AFTER_S,
    ) -> None:
        if not (math.isfinite(settle_s) and settle_s >= 0.0):
            raise ValueError("settle_s must be non-negative and finite")
        self._request = request
        self._manager = manager
        self._stitcher_rejections = stitcher_rejections
        self._clock = clock
        self._settle_s = settle_s
        self._dispatcher = RouteDispatcher(
            manager, clock, ack_timeout_s=ack_timeout_s, dropout_after_s=dropout_after_s
        )
        # Guards everything but the photo inbox, so progress() never sees a
        # half-done tick.
        self._lock = threading.RLock()
        self._inbox_lock = threading.Lock()
        self._inbox: list[ImageMessage] = []

        self._state = SurveyState.PLANNING
        self._abort_reason: str | None = None
        self._round_index = 0
        self._missions_total = 0
        self._round_done_at: float | None = None
        self._anchor: Coordinate | None = None
        self._coverage: CoverageMap | None = None
        self._unnamed_photos = 0
        # Photo keys are "drone_id/image_id": image IDs are file names, and
        # two drones can use the same ones.
        self._received_keys: set[str] = set()
        self._rejected_keys: set[str] = set()
        self._gaps_remaining = 0
        self._gaps_unplanned = 0
        self._fully_covered = False

    @property
    def state(self) -> SurveyState:
        with self._lock:
            return self._state

    def start(self) -> None:
        """
        Validates the request against the connected drones, plans round 0 and
        starts dispatching.

        Raises ``ValueError`` or ``PlanningError`` on bad input, after moving
        to ABORTED with the reason, before any command is sent.
        """

        with self._lock:
            if self._state is not SurveyState.PLANNING:
                raise RuntimeError(f"survey already {self._state.value}")
            drones = self._connected_drones()
            try:
                validate_request(self._request, drones)
                self._anchor = anchor_for_boundary(self._request.boundary)
                self._coverage = CoverageMap(
                    Polygon(boundary_to_enu(self._anchor, self._request.boundary))
                )
                routes = plan_routes(self._request, drones, round_index=0)
            except (ValueError, PlanningError) as error:
                self._abort_locked(f"refused to start: {error}")
                raise
            # Photos queued before the survey started belong to no round.
            self._take_inbox()
            self._load(routes)
            self._state = SurveyState.SURVEYING
            logger.info(
                "survey %s: %d missions for %d drones",
                self._request.survey_id,
                self._missions_total,
                len(drones),
            )

    def on_photo(self, message: ImageMessage) -> None:
        """
        Queues a photo for the next tick. Safe from any thread. Photos sent
        once the survey is final are dropped, since nothing ticks any more.
        """

        # Read without the main lock so a long tick can't block the photo
        # source; at worst one photo slips in as the survey ends.
        if self._state.is_final:
            return
        with self._inbox_lock:
            self._inbox.append(message)

    def tick(self) -> None:
        """
        Takes in photos and rejections, advances dispatch, and moves to the
        next round or state when the current round is done and settled.
        """

        with self._lock:
            if self._state not in (SurveyState.SURVEYING, SurveyState.RESHOOTING):
                self._take_inbox()  # photos after the end are dropped
                return
            self._take_photos()
            self._take_rejections()
            self._dispatcher.tick()
            if not self._dispatcher.is_done():
                self._round_done_at = None
                return
            now = self._clock()
            if self._round_done_at is None:
                self._round_done_at = now
            if now - self._round_done_at >= self._settle_s:
                # Anything that arrived during this tick still counts.
                self._take_photos()
                self._take_rejections()
                self._finish_round()

    def abort(self, reason: str = "operator abort") -> None:
        """
        Stops the survey: no more commands are sent. No effect once final.
        """

        with self._lock:
            self._abort_locked(reason)

    def progress(self) -> SurveyProgress:
        with self._lock:
            completed = self._dispatcher.completed()
            done_by: dict[str, int] = {}
            for drone_id, _ in completed:
                done_by[drone_id] = done_by.get(drone_id, 0) + 1
            dropped = self._dispatcher.dropped()
            drones = []
            for drone_id in self._dispatcher.drone_ids():
                in_flight = self._dispatcher.in_flight(drone_id)
                if drone_id in dropped:
                    activity = DroneActivity.DROPPED
                elif in_flight is not None:
                    activity = DroneActivity.FLYING
                else:
                    activity = DroneActivity.IDLE
                drones.append(
                    DroneProgress(
                        drone_id=drone_id,
                        activity=activity,
                        in_flight_task_id=None if in_flight is None else in_flight.task_id,
                        missions_done=done_by.get(drone_id, 0),
                        missions_remaining=len(self._dispatcher.remaining(drone_id)),
                    )
                )
            covered = 0.0 if self._coverage is None else self._coverage.covered_fraction()
            return SurveyProgress(
                survey_id=self._request.survey_id,
                state=self._state,
                round_index=self._round_index,
                percent_covered=100.0 * covered,
                fully_covered=self._fully_covered,
                # A photo sent twice under the same key counts once.
                photos_received=len(self._received_keys),
                # Only photos this survey received: the stitcher may hold others.
                photos_rejected=len(self._rejected_keys & self._received_keys),
                missions_done=len(completed),
                missions_total=self._missions_total,
                missions_unassigned=len(self._dispatcher.unassigned()),
                gaps_remaining=self._gaps_remaining,
                gaps_unplanned=self._gaps_unplanned,
                abort_reason=self._abort_reason,
                drones=drones,
            )

    def _abort_locked(self, reason: str) -> None:
        if self._state.is_final:
            return
        logger.warning("survey %s aborted: %s", self._request.survey_id, reason)
        self._state = SurveyState.ABORTED
        self._abort_reason = reason

    def _connected_drones(self) -> list[DroneState]:
        return [d for d in self._manager.get_all_drones().values() if d.connected]

    def _load(self, routes: dict[str, list[Mission]]) -> None:
        self._dispatcher.load(routes)
        self._missions_total += sum(len(route) for route in routes.values())
        self._round_done_at = None

    def _take_inbox(self) -> list[ImageMessage]:
        with self._inbox_lock:
            photos, self._inbox = self._inbox, []
        return photos

    def _take_photos(self) -> None:
        assert self._coverage is not None and self._anchor is not None
        for message in self._take_inbox():
            if not message.image_id:
                # A rejection can't be matched to it, so it counts as
                # received but never as coverage.
                self._unnamed_photos += 1
                self._received_keys.add(
                    _photo_key(message.drone_id, f"unnamed-{self._unnamed_photos}")
                )
                logger.info("photo from %s has no image_id", message.drone_id)
                continue
            key = _photo_key(message.drone_id, message.image_id)
            self._received_keys.add(key)
            if not message.has_valid_position():
                # One bad GPS tag must not take down the survey.
                logger.info("photo %s has no usable GPS", key)
                continue
            east_m, north_m = coordinate_to_enu(
                self._anchor, Coordinate(message.lat, message.lon, message.alt)
            )
            footprint = photo_footprint(
                east_m,
                north_m,
                message.alt,
                message.roll_deg,
                message.pitch_deg,
                message.yaw_deg,
                self._request.camera,
                max_gsd_m=self._request.max_gsd_m,
            )
            if footprint is None:
                # No GPS, no attitude, or looking at the horizon: no coverage.
                logger.info("photo %s covers no ground", key)
                continue
            self._coverage.add_photo(key, footprint)

    def _take_rejections(self) -> None:
        assert self._coverage is not None
        for drone_id, image_id, _ in self._stitcher_rejections():
            key = _photo_key(drone_id, image_id)
            if key not in self._rejected_keys:
                self._rejected_keys.add(key)
                self._coverage.mark_rejected(key)

    def _finish_round(self) -> None:
        """
        The round is flown and settled: finish, or plan the next reshoot.
        """

        assert self._coverage is not None and self._anchor is not None
        gaps = self._coverage.gaps(self._request.min_gap_area_m2)
        self._gaps_remaining = len(gaps)
        if not gaps:
            self._fully_covered = True
            self._complete()
            return
        if self._round_index >= self._request.max_reshoot_rounds:
            self._complete()
            return

        dropped = self._dispatcher.dropped()
        drones = [d for d in self._connected_drones() if d.drone_id not in dropped]
        if not drones:
            return  # wait for a drone to come back, or for an abort
        next_round = self._round_index + 1
        routes, unplanned = plan_reshoot(
            self._request, gaps, self._anchor, drones, next_round
        )
        self._gaps_unplanned = len(unplanned)
        if unplanned:
            logger.warning(
                "round %d: planner rejected %d of %d gaps",
                next_round,
                len(unplanned),
                len(gaps),
            )
        if not any(routes.values()):
            # Nothing the planner can fly: another round would plan the same.
            self._complete()
            return
        self._round_index = next_round
        self._load(routes)
        self._state = SurveyState.RESHOOTING
        logger.info(
            "round %d: reshooting %d gaps with %d missions",
            next_round,
            len(gaps) - len(unplanned),
            sum(len(route) for route in routes.values()),
        )

    def _complete(self) -> None:
        self._state = SurveyState.COMPLETE
        logger.info(
            "survey %s complete: %.1f%% covered, %d gaps left",
            self._request.survey_id,
            100.0 * self._coverage.covered_fraction() if self._coverage else 0.0,
            self._gaps_remaining,
        )


class SurveyRunner:
    """
    Calls ``executive.tick()`` at a fixed rate on its own thread until the
    survey is final or ``stop()`` is called. An error in a tick aborts the
    survey rather than leaving drones half-commanded.
    """

    def __init__(self, executive: SurveyExecutive, rate_hz: float = 2.0) -> None:
        if not (math.isfinite(rate_hz) and rate_hz > 0.0):
            raise ValueError("rate_hz must be positive and finite")
        self._executive = executive
        self._period_s = 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("runner already started")
        if self._executive.state is SurveyState.PLANNING:
            # It would tick a survey that never starts, forever.
            raise RuntimeError("start the survey before the runner")
        self._thread = threading.Thread(target=self._run, name="survey-executive")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._executive.tick()
            except Exception as error:
                logger.exception("survey tick failed")
                self._executive.abort(f"internal error: {error}")
            if self._executive.state.is_final:
                return
            self._stop.wait(self._period_s)
