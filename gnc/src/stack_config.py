from dataclasses import dataclass

from follow_controller import FollowConfig
from handoff import HandoffConfig
from hold_policy import HoldConfig
from range_rate import ReflexConfig
from safety import SafetyConfig


@dataclass(frozen=True)
class StackConfig:
    follow: FollowConfig = FollowConfig()
    safety: SafetyConfig = SafetyConfig()
    hold: HoldConfig = HoldConfig()
    reflex: ReflexConfig = ReflexConfig()
    handoff: HandoffConfig = HandoffConfig()
    target_freshness_s: float = 0.3
    command_stale_s: float = 0.75
    tree_hz: float = 2.0 
    stream_hz: float = 20.0  
    recede_speed: float = 1.0
    cmd_slew_mps2: float = 1.0

    @property
    def tree_period_ms(self) -> float:
        return 1000.0 / self.tree_hz


def _deployed() -> StackConfig:
    follow = FollowConfig(v_max=1.5, v_vertical_max=1.0, kp_vertical=0.7, a_brake=0.5)
    safety = SafetyConfig(hard_min_m=follow.hard_min_m) 
    reflex = ReflexConfig(hard_min_m=follow.hard_min_m)
    return StackConfig(follow=follow, safety=safety, reflex=reflex)

DEPLOYED = _deployed()
