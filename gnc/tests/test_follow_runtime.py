import math

from follow_runtime import (
    LatestObservationController,
    RuntimeObservation,
    choose_ema_alpha,
    choose_stream_hz,
)


def observation(z, sequence, capture, receive=None, x=0.0, y=0.0):
    return RuntimeObservation(
        x_m=x,
        y_m=y,
        z_m=z,
        track_id=4,
        sequence_num=sequence,
        capture_time_s=capture,
        receive_time_s=capture if receive is None else receive,
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


def test_deployed_rate_and_filter_derivation_rules():
    assert choose_stream_hz(7.0) == 20.0
    assert choose_stream_hz(21.0) == 25.0
    assert choose_stream_hz(80.0) == 50.0
    assert choose_ema_alpha(0.05) == 0.5
    assert choose_ema_alpha(0.1) == 0.7
