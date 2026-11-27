import random
from dataclasses import replace

import obstacle_avoidance.avoidance as avoidance_module
import pytest
from obstacle_avoidance import ObstacleAvoider
from obstacle_avoidance.avoidance import (
    _MovingTraffic,
    _segment_is_safe,
)
from obstacle_avoidance.models import (
    AvoidanceRequest,
    PlanStatus,
    Point2D,
    TrafficAircraft,
)
from obstacle_avoidance.patrol import BackAndForthHistory, BackAndForthPrediction

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

HERE = Coordinate(43.0, -80.0, 10.0)


def _traffic(east_m: float, north_m: float) -> TrafficAircraft:
    position = enu_to_coordinate(HERE, east_m, north_m, HERE.alt)
    return TrafficAircraft(
        aircraft_index=1,
        lat=position.lat,
        lon=position.lon,
        altitude_agl_m=10.0,
        speed_mps=1.0,
        heading_deg=90.0,
        horizontal_keep_away_m=2.0,
        vertical_keep_away_m=5.0,
    )


def _back_and_forth_positions() -> list[int]:
    return (
        list(range(0, 11))
        + list(range(9, -1, -1))
        + list(range(1, 11))
        + list(range(9, -1, -1))
        + list(range(1, 11))
        + [9, 8, 7]
    )


def test_pattern_requires_two_repeatable_cycles_and_reflects_at_endpoints() -> None:
    history = BackAndForthHistory()
    positions = _back_and_forth_positions()
    prediction = None

    for timestamp_s, east_m in enumerate(positions):
        prediction = history.observe(
            (_traffic(east_m, 0.0),),
            sample_time_s=float(timestamp_s),
            origin_lat=HERE.lat,
            origin_lon=HERE.lon,
        ).get(1)
        if timestamp_s < 53:
            assert prediction is None

    assert prediction is not None
    assert prediction.period_s == 20.0
    assert prediction.direction == -1
    assert prediction.position_at(0.0).x == pytest.approx(
        east_north_coordinate_offset_m(
            HERE.lat,
            HERE.lon,
            _traffic(7.0, 0.0).lat,
            _traffic(7.0, 0.0).lon,
        )[0],
        abs=0.1,
    )
    assert prediction.uncertainty_at(5.0) >= prediction.uncertainty_at(0.0)


def test_polygon_trajectory_is_not_classified_as_back_and_forth() -> None:
    history = BackAndForthHistory()
    corners = ((0, 0), (10, 0), (10, 10), (0, 10), (0, 0))
    positions: list[tuple[int, int]] = []
    for _ in range(3):
        for (start_x, start_y), (end_x, end_y) in zip(corners, corners[1:]):
            steps = max(abs(end_x - start_x), abs(end_y - start_y))
            for step in range(steps):
                fraction = step / steps
                positions.append(
                    (
                        round(start_x + (end_x - start_x) * fraction),
                        round(start_y + (end_y - start_y) * fraction),
                    )
                )
    positions.append(corners[0])

    prediction = None
    for timestamp_s, (east_m, north_m) in enumerate(positions):
        prediction = history.observe(
            (_traffic(east_m, north_m),),
            sample_time_s=float(timestamp_s),
            origin_lat=HERE.lat,
            origin_lon=HERE.lon,
        ).get(1)

    assert prediction is None


def test_stale_history_is_not_used_to_predict_a_patrol() -> None:
    history = BackAndForthHistory()
    positions = _back_and_forth_positions()
    for timestamp_s, east_m in enumerate(positions):
        history.observe(
            (_traffic(east_m, 0.0),),
            sample_time_s=float(timestamp_s),
            origin_lat=HERE.lat,
            origin_lon=HERE.lon,
        )

    prediction = history.observe(
        (_traffic(8.0, 0.0),),
        sample_time_s=56.0,
        origin_lat=HERE.lat,
        origin_lon=HERE.lon,
    )

    assert prediction == {}


def test_motion_outside_learned_endpoints_invalidates_the_pattern() -> None:
    history = BackAndForthHistory()
    positions = _back_and_forth_positions()
    for timestamp_s, east_m in enumerate(positions):
        history.observe(
            (_traffic(east_m, 0.0),),
            sample_time_s=float(timestamp_s),
            origin_lat=HERE.lat,
            origin_lon=HERE.lon,
        )

    prediction = history.observe(
        (_traffic(15.0, 0.0),),
        sample_time_s=54.0,
        origin_lat=HERE.lat,
        origin_lon=HERE.lon,
    )

    assert prediction == {}


def test_reset_clears_learned_patrol_history() -> None:
    history = BackAndForthHistory()
    positions = _back_and_forth_positions()
    for timestamp_s, east_m in enumerate(positions):
        history.observe(
            (_traffic(east_m, 0.0),),
            sample_time_s=float(timestamp_s),
            origin_lat=HERE.lat,
            origin_lon=HERE.lon,
        )

    history.reset()
    prediction = history.observe(
        (_traffic(7.0, 0.0),),
        sample_time_s=54.0,
        origin_lat=HERE.lat,
        origin_lon=HERE.lon,
    )

    assert prediction == {}


def test_time_aware_segment_check_uses_repeated_motion_not_current_heading() -> None:
    patrol = BackAndForthPrediction(
        center=Point2D(0.0, 0.0),
        axis_x=0.0,
        axis_y=1.0,
        lower_m=12.0,
        upper_m=32.0,
        current_m=12.0,
        direction=1,
        speed_mps=2.0,
        period_s=20.0,
        uncertainty_m=0.5,
        uncertainty_growth_mps=0.0,
    )
    moving = _MovingTraffic(
        position=Point2D(0.0, 12.0),
        east_mps=0.0,
        north_mps=-2.0,
        radius_m=8.0,
        patrol=patrol,
    )
    linear = _MovingTraffic(
        position=Point2D(0.0, 12.0),
        east_mps=0.0,
        north_mps=-2.0,
        radius_m=8.0,
    )

    assert _segment_is_safe(
        Point2D(0.0, 0.0),
        Point2D(0.0, 16.0),
        (moving,),
        own_speed_mps=2.0,
        horizon_s=20.0,
    )
    assert not _segment_is_safe(
        Point2D(0.0, 0.0),
        Point2D(0.0, 16.0),
        (linear,),
        own_speed_mps=2.0,
        horizon_s=20.0,
    )


def test_confirmed_patrol_uses_time_checks_not_the_whole_patrol_corridor() -> None:
    avoider = ObstacleAvoider()
    goal = enu_to_coordinate(HERE, 0.0, 16.0, HERE.alt)
    positions = _back_and_forth_positions()
    plan = None

    for timestamp_s, east_m in enumerate(positions):
        plan = avoider.plan(
            AvoidanceRequest(
                position=HERE,
                goal=goal,
                traffic=(_traffic(east_m, 30.0),),
                traffic_age_s=0.0,
                traffic_sample_time_s=float(timestamp_s),
            )
        )

    assert plan is not None
    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.reason == "PATROL_PATTERN_ACTIVE"
    assert plan.obstacle_count == 0
    avoider.reset_planner()
    plan = avoider.plan(
        AvoidanceRequest(
            position=HERE,
            goal=goal,
            traffic=(_traffic(7.0, 30.0),),
            traffic_age_s=0.0,
            traffic_sample_time_s=float(len(positions)),
        )
    )
    assert plan.reason == "PATROL_PATTERN_ACTIVE"


def test_unusable_patrol_plan_falls_back_to_the_linear_planner(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_segment_is_safe = avoidance_module._segment_is_safe

    def reject_patrol_segments(start, end, traffic, **kwargs):
        if any(aircraft.patrol is not None for aircraft in traffic):
            return False
        return original_segment_is_safe(start, end, traffic, **kwargs)

    monkeypatch.setattr(
        avoidance_module, "_segment_is_safe", reject_patrol_segments
    )
    avoider = ObstacleAvoider()
    goal = enu_to_coordinate(HERE, 0.0, 16.0, HERE.alt)
    positions = _back_and_forth_positions()
    plan = None

    for timestamp_s, east_m in enumerate(positions):
        plan = avoider.plan(
            AvoidanceRequest(
                position=HERE,
                goal=goal,
                traffic=(_traffic(east_m, 30.0),),
                traffic_age_s=0.0,
                traffic_sample_time_s=float(timestamp_s),
            )
        )

    assert plan is not None
    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.reason == "PATROL_FALLBACK"


def _patrol_truth_m(time_s: float, span_m: float, speed_mps: float, pause_s: float) -> float:
    """East position of an aircraft flying 0..span_m and back, pausing at each end."""

    leg_s = span_m / speed_mps
    phase_s = time_s % (2.0 * (leg_s + pause_s))
    if phase_s < leg_s:
        return speed_mps * phase_s
    if phase_s < leg_s + pause_s:
        return span_m
    if phase_s < 2.0 * leg_s + pause_s:
        return span_m - speed_mps * (phase_s - leg_s - pause_s)
    return 0.0


def _patrolling_traffic(east_m: float, speed_mps: float) -> TrafficAircraft:
    position = enu_to_coordinate(HERE, east_m, 0.0, HERE.alt)
    return replace(
        _traffic(east_m, 0.0), lat=position.lat, lon=position.lon, speed_mps=speed_mps
    )


@pytest.mark.parametrize(
    ("span_m", "speed_mps", "pause_s"),
    [(40.0, 5.0, 0.0), (40.0, 5.0, 1.0), (60.0, 10.0, 0.0), (100.0, 15.0, 0.0), (100.0, 15.0, 1.0)],
)
def test_jittery_1hz_samples_of_a_fast_patrol_stay_within_the_modeled_uncertainty(
    span_m: float, speed_mps: float, pause_s: float
) -> None:
    rng = random.Random(1)
    history = BackAndForthHistory()
    sample_time_s = rng.uniform(0.0, 1.0)
    detected_at_s = None

    while sample_time_s < 170.0:
        east_m = _patrol_truth_m(sample_time_s, span_m, speed_mps, pause_s)
        prediction = history.observe(
            (_patrolling_traffic(east_m, speed_mps),),
            sample_time_s=sample_time_s,
            origin_lat=HERE.lat,
            origin_lon=HERE.lon,
        ).get(1)
        if prediction is None:
            # Once confirmed, sampling jitter alone must not drop the pattern.
            assert detected_at_s is None
        else:
            detected_at_s = detected_at_s or sample_time_s
            for step in range(41):
                elapsed_s = step * 0.5
                truth_m = _patrol_truth_m(
                    sample_time_s + elapsed_s, span_m, speed_mps, pause_s
                )
                error_m = abs(prediction.position_at(elapsed_s).x - truth_m)
                assert error_m <= prediction.uncertainty_at(elapsed_s) + 0.5
        sample_time_s += 1.0 + rng.uniform(-0.1, 0.1)

    assert detected_at_s is not None


def test_a_stale_traffic_snapshot_does_not_erase_a_learned_patrol() -> None:
    avoider = ObstacleAvoider()
    goal = enu_to_coordinate(HERE, 0.0, 16.0, HERE.alt)
    positions = _back_and_forth_positions()

    def plan_at(sample_time_s: float, east_m: int, age_s: float = 0.0):
        return avoider.plan(
            AvoidanceRequest(
                position=HERE,
                goal=goal,
                traffic=(_traffic(east_m, 30.0),),
                traffic_age_s=age_s,
                traffic_sample_time_s=sample_time_s,
            )
        )

    for timestamp_s, east_m in enumerate(positions):
        plan = plan_at(float(timestamp_s), east_m)
    assert plan.reason == "PATROL_PATTERN_ACTIVE"

    last_s = float(len(positions) - 1)
    assert plan_at(last_s, positions[-1], age_s=2.0).reason == "STALE_TRAFFIC"
    assert plan_at(last_s + 1.0, 6).reason == "PATROL_PATTERN_ACTIVE"


def _moving_traffic_for(aircraft: TrafficAircraft, patrol_buffer_m: float, patrol):
    return avoidance_module._moving_traffic(
        (aircraft,),
        origin_lat=HERE.lat,
        origin_lon=HERE.lon,
        age_s=0.0,
        own_speed_mps=2.0,
        clearance_margin_m=1.0,
        patrol_predictions={} if patrol is None else {1: patrol},
        patrol_buffer_m=patrol_buffer_m,
    )[0]


def test_confirmed_patrol_uses_the_configured_buffer_not_the_speed_buffer() -> None:
    aircraft = replace(_traffic(0.0, 0.0), speed_mps=15.0)
    patrol = BackAndForthPrediction(
        center=Point2D(0.0, 0.0), axis_x=1.0, axis_y=0.0, lower_m=-50.0,
        upper_m=50.0, current_m=0.0, direction=1, speed_mps=15.0, period_s=13.3,
        uncertainty_m=3.0, uncertainty_growth_mps=0.0,
    )

    linear = _moving_traffic_for(aircraft, 6.0, None)
    patrolling = _moving_traffic_for(aircraft, 6.0, patrol)
    tighter = _moving_traffic_for(aircraft, 2.0, patrol)

    # keep-away 2 + margin 1 + live buffer 18 at 15 m/s.
    assert linear.radius_m == pytest.approx(21.0)
    # keep-away 2 + margin 1 + patrol buffer + model uncertainty 3.
    assert patrolling.radius_m == pytest.approx(12.0)
    assert tighter.radius_m == pytest.approx(8.0)


def test_patrol_buffer_must_be_non_negative() -> None:
    with pytest.raises(ValueError):
        avoidance_module.AvoidanceConfig(patrol_traffic_buffer_m=-1.0)


def test_patrol_model_uncertainty_can_be_left_out_of_the_circle() -> None:
    aircraft = replace(_traffic(0.0, 0.0), speed_mps=15.0)
    patrol = BackAndForthPrediction(
        center=Point2D(0.0, 0.0), axis_x=1.0, axis_y=0.0, lower_m=-50.0,
        upper_m=50.0, current_m=0.0, direction=1, speed_mps=15.0, period_s=13.3,
        uncertainty_m=3.0, uncertainty_growth_mps=1.0,
    )

    motion = avoidance_module._moving_traffic(
        (aircraft,),
        origin_lat=HERE.lat,
        origin_lon=HERE.lon,
        age_s=0.0,
        own_speed_mps=2.0,
        clearance_margin_m=1.0,
        patrol_predictions={1: patrol},
        patrol_buffer_m=0.0,
        patrol_model_uncertainty=False,
    )[0]

    # keep-away 2 + margin 1 only, now and later in a crossing.
    assert motion.radius_m == pytest.approx(3.0)
    assert avoidance_module._traffic_radius_at(motion, 10.0) == pytest.approx(3.0)


def test_patrol_defaults_trust_the_model_without_extra_buffer() -> None:
    config = avoidance_module.AvoidanceConfig()

    assert config.patrol_traffic_buffer_m == 0.0
    assert config.patrol_model_uncertainty is False
