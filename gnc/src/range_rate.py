"""Filtered range/closing-rate estimation for the proximity reflexes"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


@dataclass(frozen=True)
class RangeRateConfig:
    window: int = 6 
    max_sample_age_s: float = 0.6  
    min_dt_s: float = 0.02  # rejects burst-delivery dt spikes


class RangeRateEstimator:
    def __init__(self, config: RangeRateConfig = RangeRateConfig()) -> None:
        self.config = config
        self._samples: Deque[Tuple[float, float]] = deque(maxlen=config.window)

    def reset(self) -> None:
        self._samples.clear()

    def add(self, range_m: float, t_s: float) -> None:
        """Record one range sample; near-simultaneous re-deliveries are dropped."""
        if self._samples and (t_s - self._samples[-1][0]) < self.config.min_dt_s:
            return
        self._samples.append((t_s, range_m))

    def closing_rate(self, now_s: float) -> float:
        """Least-squares closing rate over the fresh window."""
        fresh = [
            (t, r) for t, r in self._samples if (now_s - t) <= self.config.max_sample_age_s
        ]
        if len(fresh) < 2:
            return 0.0
        n = float(len(fresh))
        mean_t = sum(t for t, _ in fresh) / n
        mean_r = sum(r for _, r in fresh) / n
        var_t = sum((t - mean_t) ** 2 for t, _ in fresh)
        if var_t <= 0.0:
            return 0.0
        cov_tr = sum((t - mean_t) * (r - mean_r) for t, r in fresh)
        slope = cov_tr / var_t  # d(range)/dt; negative while approaching
        return -slope


@dataclass(frozen=True)
class ReflexConfig:
    hard_min_m: float = 1.5
    a_brake: float = 1.0
    reaction_time_s: float = 0.3
    danger_ticks: int = 2  # consecutive predictive positives required
    rate: RangeRateConfig = RangeRateConfig(max_sample_age_s=0.35)


class ReflexMonitor:
    """Fast hard-min + filtered closing-rate danger check"""

    def __init__(self, config: ReflexConfig = ReflexConfig()) -> None:
        self.config = config
        self._estimator = RangeRateEstimator(config.rate)
        self._streak = 0

    def reset(self) -> None:
        self._estimator.reset()
        self._streak = 0

    def update(self, range_m: Optional[float], t_s: float) -> bool:
        """Feed the freshest range (None = no fresh target); True = danger."""
        cfg = self.config
        if range_m is None:
            # No fresh target: the tree owns lost-target policy; the reflex stands down.
            self._streak = 0
            return False

        self._estimator.add(range_m, t_s)

        # Hard-min breach: instant, undebounced
        if range_m < cfg.hard_min_m:
            return True

        closing = self._estimator.closing_rate(t_s)
        if closing > 0.0:
            stop_dist = (
                (closing * closing) / (2.0 * cfg.a_brake) if cfg.a_brake > 0.0 else math.inf
            )
            predicted_min = range_m - closing * cfg.reaction_time_s - stop_dist
            if predicted_min < cfg.hard_min_m:
                self._streak += 1
                return self._streak >= cfg.danger_ticks
        self._streak = 0
        return False
