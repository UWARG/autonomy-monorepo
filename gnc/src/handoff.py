"""SITL auto-handoff sequencer: GUIDED -> arm -> takeoff -> airborne"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class HandoffAction(Enum):
    WAIT = "wait" 
    REQUEST_GUIDED = "request_guided"
    REQUEST_ARM = "request_arm"
    REQUEST_TAKEOFF = "request_takeoff"
    COMPLETE = "complete"  


@dataclass(frozen=True)
class HandoffConfig:
    takeoff_alt_m: float = 3.0
    alt_fraction: float = 0.95
    climb_timeout_s: float = 20.0 
    arm_retry_s: float = 2.0 


class HandoffSequencer:
    def __init__(self, config: HandoffConfig = HandoffConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._last_arm_s: Optional[float] = None
        self._last_takeoff_req_s: Optional[float] = None
        self._takeoff_sent_s: Optional[float] = None
        self._complete = False

    @property
    def is_complete(self) -> bool:
        return self._complete

    def mark_complete(self) -> None:
        self._complete = True

    def notify_takeoff_sent(self, now_s: float) -> None:
        self._takeoff_sent_s = now_s

    def step(
        self, connected: bool, mode: str, armed: bool, alt_m: float, now_s: float
    ) -> HandoffAction:
        if self._complete:
            return HandoffAction.COMPLETE
        if not connected:
            return HandoffAction.WAIT
        if mode != "GUIDED":
            return HandoffAction.REQUEST_GUIDED
        if not armed:
            if self._last_arm_s is None or (now_s - self._last_arm_s) >= self.config.arm_retry_s:
                self._last_arm_s = now_s
                return HandoffAction.REQUEST_ARM
            return HandoffAction.WAIT
        if self._takeoff_sent_s is None:
            if (
                self._last_takeoff_req_s is None
                or (now_s - self._last_takeoff_req_s) >= self.config.arm_retry_s
            ):
                self._last_takeoff_req_s = now_s
                return HandoffAction.REQUEST_TAKEOFF
            return HandoffAction.WAIT
        airborne = alt_m >= self.config.alt_fraction * self.config.takeoff_alt_m
        if airborne:
            self._complete = True
            return HandoffAction.COMPLETE
        if (now_s - self._takeoff_sent_s) < self.config.climb_timeout_s:
            return HandoffAction.WAIT
        
        self._takeoff_sent_s = None
        self._last_takeoff_req_s = None
        return HandoffAction.WAIT
