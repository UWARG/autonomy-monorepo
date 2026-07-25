import math

from follow_runtime import (
    LatestObservationController,
    RuntimeObservation,
    choose_ema_alpha,
    choose_stream_hz,
)


def observation(
    z,
    sequence,
    capture,
    receive=None,
    x=0.0,
    y=0.0,
    detector_confirmed=True,
    detector_capture=None,
    detector_sequence=None,
    spatial_control_valid=True,
):
    return RuntimeObservation(
        x_m=x,
        y_m=y,
        z_m=z,
        track_id=4,
        sequence_num=sequence,
        capture_time_s=capture,
        receive_time_s=capture if receive is None else receive,
        detector_capture_time_s=detector_capture,
        detector_sequence_num=detector_sequence,
        detector_confirmed=detector_confirmed,
        spatial_control_valid=spatial_control_valid,
    )


def test_capture_age_not_receive_age_controls_freshness():
    runtime = LatestObservationController(freshness_s=0.3)
    runtime.update(observation(3.0, 1, capture=10.0, receive=10.25))
    assert runtime.evaluate(10.29).fresh
    assert not runtime.evaluate(10.31).fresh


def test_raw_hard_min_preempts_even_when_ema_is_still_far():
    runtime = LatestObservationController(ema_alpha=0.1)
    runtime.update(observation(4.0, 1, 10.0))
    runtime.update(observation(1.4, 2, 10.05))
    output = runtime.evaluate(10.05)
    assert output.proximity_emergency
    assert output.raw_range_m == 1.4
    assert output.filtered_xyz_m[2] > 3.0
    assert output.setpoint.range_3d_m > 3.0


def test_ema_applies_only_once_per_sequence():
    runtime = LatestObservationController(ema_alpha=0.5)
    assert runtime.update(observation(4.0, 1, 10.0))
    assert runtime.update(observation(2.0, 2, 10.05))
    assert not runtime.update(observation(1.0, 2, 10.05))
    assert runtime.evaluate(10.05).filtered_xyz_m[2] == 3.0


def test_metrics_use_capture_periods_latency_and_sequence_gaps():
    runtime = LatestObservationController()
    runtime.update(observation(3.0, 10, 1.00, 1.01))
    runtime.update(observation(3.0, 11, 1.05, 1.07))
    runtime.update(observation(3.0, 13, 1.10, 1.20))
    metrics = runtime.metrics()
    assert math.isclose(metrics.effective_fps, 20.0)
    assert metrics.sequence_gaps == 1
    assert metrics.latency_p99_s > 0.09


def test_tracker_only_frame_cannot_refresh_stale_detector_xyz():
    runtime = LatestObservationController(freshness_s=0.3, detector_stride=2)
    assert runtime.update(
        observation(
            3.0,
            10,
            capture=10.0,
            detector_capture=10.0,
            detector_sequence=10,
        )
    )
    assert not runtime.update(
        observation(
            1.4,
            11,
            capture=10.25,
            detector_confirmed=False,
            detector_capture=10.0,
            detector_sequence=10,
        )
    )
    output = runtime.evaluate(10.31)
    assert not output.fresh
    assert output.raw_range_m == 3.0
    assert not output.proximity_emergency
    assert runtime.metrics().tracker_only_frames == 1


def test_tracker_only_frame_does_not_update_closing_rate():
    runtime = LatestObservationController(detector_stride=2)
    runtime.update(
        observation(
            3.0,
            10,
            1.0,
            detector_capture=1.0,
            detector_sequence=10,
        )
    )
    runtime.update(
        observation(
            1.0,
            11,
            1.05,
            detector_confirmed=False,
            detector_capture=1.0,
            detector_sequence=10,
        )
    )
    output = runtime.evaluate(1.05)
    assert output.raw_range_m == 3.0
    assert not output.proximity_emergency


def test_out_of_range_detector_counts_timing_without_refreshing_control():
    runtime = LatestObservationController(freshness_s=0.3)
    runtime.update(observation(2.9, 1, 1.0))
    assert not runtime.update(observation(3.1, 2, 1.1, spatial_control_valid=False))
    assert runtime.evaluate(1.31).raw_range_m == 2.9
    assert not runtime.evaluate(1.31).fresh
    assert math.isclose(runtime.metrics().detector_fps, 10.0)


def test_deployed_rate_and_filter_derivation_rules():
    assert choose_stream_hz(7.0) == 20.0
    assert choose_stream_hz(21.0) == 25.0
    assert choose_stream_hz(80.0) == 50.0
    assert choose_ema_alpha(0.05) == 0.5
    assert choose_ema_alpha(0.1) == 0.7
