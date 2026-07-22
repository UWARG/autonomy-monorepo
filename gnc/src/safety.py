"""Autonomous safety monitor for the follow controller"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import math

from range_rate import RangeRateConfig, RangeRateEstimator


class EStopAction(Enum):
    NONE = "none"  # follow normally
    ZERO_VELOCITY = "zero_velocity"  # stop / hold (instant, GPS-independent)
    BRAKE = "brake"  # authoritative latch (needs valid EKF)


@dataclass(frozen=True)
class SafetyConfig:
    hard_min_m: float = 1.5
    lost_timeout_s: float = 1.0
    a_brake: float = 1.0
    reaction_time_s: float = 0.3
    clear_margin_m: float = 0.3
    clear_ticks: int = 3


@dataclass(frozen=True)
class SafetyVerdict:
    action: EStopAction
    is_emergency: bool
    recede: bool  # actively back away vs. just hold
    reason: str

class SafetyMonitor:
    def __init__(self, config: SafetyConfig = SafetyConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._time_since_seen_s = 0.0
        self._t_s = 0.0  # internal clock accumulated from dt
        self._rate = RangeRateEstimator(RangeRateConfig())
        self._latched = False
        self._clear_count = 0

    def _latch(self, action: EStopAction, recede: bool, reason: str) -> SafetyVerdict:
        self._latched = True
        self._clear_count = 0
        return SafetyVerdict(action=action, is_emergency=True, recede=recede, reason=reason)

    def evaluate(self, range_3d_m: Optional[float], dt_s: float) -> SafetyVerdict:
        """Evaluate one tick. ``range_3d_m`` is None when no target was seen."""
        cfg = self.config

        self._t_s += dt_s

        # --- target lost / brief dropout ---
        if range_3d_m is None:
            self._time_since_seen_s += dt_s
            self._rate.reset()
            if self._time_since_seen_s > cfg.lost_timeout_s:
                return self._latch(
                    EStopAction.ZERO_VELOCITY, recede=False, reason="target lost"
                )
            # brief dropout: hold but do not latch an emergency yet
            return SafetyVerdict(
                action=EStopAction.ZERO_VELOCITY,
                is_emergency=False,
                recede=False,
                reason="target dropout (holding)",
            )

        # target present this tick: least-squares closing rate over a short window
        self._time_since_seen_s = 0.0
        self._rate.add(range_3d_m, self._t_s)
        closing_rate = self._rate.closing_rate(self._t_s)

        # --- hard-min breach ---
        if range_3d_m < cfg.hard_min_m:
            return self._latch(
                EStopAction.ZERO_VELOCITY, recede=True, reason="hard-min breach"
            )

        # --- closing too fast to stop before the ring ---
        if closing_rate > 0.0:
            reaction_dist = closing_rate * cfg.reaction_time_s
            stopping_dist = (closing_rate * closing_rate) / (2.0 * cfg.a_brake)
            predicted_min = range_3d_m - reaction_dist - stopping_dist
            if predicted_min < cfg.hard_min_m:
                return self._latch(
                    EStopAction.ZERO_VELOCITY,
                    recede=True,
                    reason="closing too fast to stop before hard-min",
                )

        # --- clearing a previously latched emergency ---
        if self._latched:
            if range_3d_m >= cfg.hard_min_m + cfg.clear_margin_m:
                self._clear_count += 1
                if self._clear_count >= cfg.clear_ticks:
                    self._latched = False
                    self._clear_count = 0
                    return SafetyVerdict(
                        EStopAction.NONE, is_emergency=False, recede=False, reason="cleared"
                    )
            else:
                self._clear_count = 0
            return SafetyVerdict(
                EStopAction.ZERO_VELOCITY,
                is_emergency=True,
                recede=False,
                reason="latched -- awaiting stable re-acquire",
            )

        return SafetyVerdict(EStopAction.NONE, is_emergency=False, recede=False, reason="ok")


def stopping_distance(speed: float, a_brake: float) -> float:
    """Distance needed to brake from ``speed`` to zero at ``a_brake`` (m)."""
    if a_brake <= 0.0:
        return math.inf
    return (speed * speed) / (2.0 * a_brake)
