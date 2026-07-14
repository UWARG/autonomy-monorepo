"""Pure steady-state position-hold policy for the setpoint streamer"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

class HoldAction(Enum):
    FOLLOW = "follow"  # stream the velocity command
    LATCH = "latch"  
    HOLD = "hold"  


@dataclass(frozen=True)
class HoldConfig:
    enter_speed_mps: float = 0.08  
    exit_speed_mps: float = 0.25  
    enter_yaw_rate: float = 0.10  
    exit_yaw_rate: float = 0.40 # rad/s
    min_hold_s: float = 2.0 
    relatch_cooldown_s: float = 1.0 
    pose_max_age_s: float = 0.5 


@dataclass(frozen=True)
class HoldStatus:
    action: HoldAction
    relatch_count: int
    reason: str


class HoldPolicy:
    """Hold/follow state machine. Feed it every streamer tick while following."""
    def __init__(self, config: HoldConfig = HoldConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        """Forget everything."""
        self._holding = False
        self._latch_s: Optional[float] = None
        self._release_s: Optional[float] = None
        self._relatch_count = 0

    @property
    def is_holding(self) -> bool:
        return self._holding

    @property
    def relatch_count(self) -> int:
        return self._relatch_count

    def _status(self, action: HoldAction, reason: str) -> HoldStatus:
        return HoldStatus(action=action, relatch_count=self._relatch_count, reason=reason)

    def step(self, speed_mps: float, yaw_rate: float, pose_age_s: float, ekf_ok: bool, now_s: float) -> HoldStatus:
        cfg = self.config

        if self._holding:
            # A degraded EKF invalidates the latched estimate: release immediately.
            if not ekf_ok:
                self._holding = False
                self._release_s = now_s
                return self._status(HoldAction.FOLLOW, "released: ekf degraded")
            in_dwell = self._latch_s is not None and (now_s - self._latch_s) < cfg.min_hold_s
            moved = speed_mps > cfg.exit_speed_mps or abs(yaw_rate) > cfg.exit_yaw_rate
            if moved and not in_dwell:
                self._holding = False
                self._release_s = now_s
                return self._status(HoldAction.FOLLOW, "released: target moved")
            reason = "holding (dwell)" if in_dwell else "holding"
            return self._status(HoldAction.HOLD, reason)

        settled = speed_mps < cfg.enter_speed_mps and abs(yaw_rate) < cfg.enter_yaw_rate
        if not settled:
            return self._status(HoldAction.FOLLOW, "following")
        if not ekf_ok:
            return self._status(HoldAction.FOLLOW, "settled but ekf degraded")
        if pose_age_s >= cfg.pose_max_age_s:
            return self._status(HoldAction.FOLLOW, "settled but pose stale")
        in_cooldown = (
            self._release_s is not None and (now_s - self._release_s) < cfg.relatch_cooldown_s
        )
        if in_cooldown:
            return self._status(HoldAction.FOLLOW, "settled but in re-latch cooldown")

        self._holding = True
        self._latch_s = now_s
        self._relatch_count += 1
        return self._status(HoldAction.LATCH, "latched")
