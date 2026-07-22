"""Gate 0 -- filtered closing-rate estimation + the streamer reflex."""

import random

from hypothesis import given, settings
from hypothesis import strategies as st

from range_rate import RangeRateConfig, RangeRateEstimator, ReflexConfig, ReflexMonitor

RATE_CFG = RangeRateConfig(window=6, max_sample_age_s=0.6, min_dt_s=0.02)
REFLEX_CFG = ReflexConfig(hard_min_m=1.5, a_brake=1.0, reaction_time_s=0.3, danger_ticks=2)


def test_estimator_recovers_constant_closing_rate():
    est = RangeRateEstimator(RATE_CFG)
    for i in range(6):
        t = i * 0.05
        est.add(5.0 - 2.0 * t, t)  # approaching at 2 m/s
    assert abs(est.closing_rate(0.25) - 2.0) < 1e-9


def test_estimator_zero_with_fewer_than_two_samples():
    est = RangeRateEstimator(RATE_CFG)
    assert est.closing_rate(0.0) == 0.0
    est.add(5.0, 0.0)
    assert est.closing_rate(0.0) == 0.0


def test_estimator_rejects_burst_delivery_spike():
    """Two samples 1 ms apart (burst delivery) must not produce a huge rate."""
    est = RangeRateEstimator(RATE_CFG)
    est.add(5.00, 0.000)
    est.add(4.96, 0.001)  # would be 40 m/s from a naive single-tick delta
    assert est.closing_rate(0.001) == 0.0  # second sample dropped by min_dt guard


def test_estimator_ignores_stale_samples():
    est = RangeRateEstimator(RATE_CFG)
    est.add(5.0, 0.0)
    est.add(4.9, 0.05)
    # much later: the old window has aged out entirely -> no vote
    assert est.closing_rate(2.0) == 0.0


def test_estimator_smooths_noise_vs_single_tick_delta():
    rng = random.Random(1)
    est = RangeRateEstimator(RATE_CFG)
    t = 0.0
    prev = None
    single_tick_max = 0.0
    filtered_max = 0.0
    for i in range(100):
        r = 5.0 + rng.gauss(0.0, 0.04)  # stationary target, 4 cm range noise
        if prev is not None:
            single_tick_max = max(single_tick_max, abs((prev - r) / 0.05))
        est.add(r, t)
        filtered_max = max(filtered_max, abs(est.closing_rate(t)))
        prev = r
        t += 0.05
    assert filtered_max < single_tick_max / 2  # the fit is materially quieter
    assert filtered_max < 1.0  # and never looks like a real lunge


def test_reflex_hard_min_is_instant():
    reflex = ReflexMonitor(REFLEX_CFG)
    assert reflex.update(1.2, 0.0) is True  # first-ever sample inside the ring trips


def test_reflex_predictive_needs_consecutive_ticks():
    reflex = ReflexMonitor(REFLEX_CFG)
    t = 0.0
    fired_at = None
    # A genuine 3 m/s lunge from 4 m: stopping distance 4.5 m + reaction 0.9 m >> range.
    for i in range(10):
        r = 4.0 - 3.0 * t
        if reflex.update(r, t) and fired_at is None:
            fired_at = i
        t += 0.05
    assert fired_at is not None  # the lunge is caught...
    assert fired_at >= 2  # ...but only after the debounce, not on the first delta


def test_reflex_no_fresh_target_stands_down():
    reflex = ReflexMonitor(REFLEX_CFG)
    reflex.update(4.0, 0.0)
    assert reflex.update(None, 0.05) is False


def test_reflex_receding_never_fires():
    reflex = ReflexMonitor(REFLEX_CFG)
    t = 0.0
    for i in range(50):
        assert reflex.update(3.0 + 0.5 * t, t) is False  # walking away
        t += 0.05


@settings(max_examples=200)
@given(st.integers(min_value=0, max_value=10_000))
def test_property_reflex_never_fires_on_noisy_recede(seed):
    """A monotonically-receding target with realistic noise must never trip."""
    rng = random.Random(seed)
    reflex = ReflexMonitor(REFLEX_CFG)
    t = 0.0
    for i in range(80):
        r = 3.0 + 0.4 * t + rng.gauss(0.0, 0.03)
        assert reflex.update(r, t) is False
        t += 0.05
