"""Pure flight-action decision for the setpoint streamer
Ladder (highest precedence first):
  1. Not in GUIDED        -> RELEASE        (pilot has control; do not command)
  2. Proximity danger     -> SET_BRAKE      (hard latch) if EKF ok, else STREAM_ZERO
  3. Target lost (sticky) -> SET_LOITER     (hold the spot) if EKF ok, else STREAM_ZERO
  4. Stale / brief hold   -> HOLD_POSITION if a position hold is latched (and the
                             EKF is healthy) else STREAM_ZERO. Keeping the latched
                             pose through a brief dropout is the anti-drift fix:
                             dropping to zero velocity for the stale half of every
                             command period re-latched the hold at whatever altitude
                             the vehicle had sagged to (the re-latch ratchet).
  5. Nominal              -> STREAM_VELOCITY (publish the follow command)
"""

from dataclasses import dataclass
from enum import Enum


class ControlAction(Enum):
    STREAM_VELOCITY = "stream_velocity" 
    STREAM_ZERO = "stream_zero"  
    HOLD_POSITION = "hold_position"  # republish the latched position hold
    SET_BRAKE = "set_brake"  # hard latch
    SET_LOITER = "set_loiter"  # steady-state / lost hold
    RELEASE = "release"


@dataclass(frozen=True)
class FlightInputs:
    in_guided: bool
    ekf_ok: bool
    command_fresh: bool  
    reflex_danger: bool 
    estop_emergency: bool  
    estop_recede: bool  
    estop_hold: bool
    holding: bool = False
    armed: bool = True


def decide(i: FlightInputs) -> ControlAction:
    """Choose the streamer action for this tick"""
    # 1. Pilot took over: do not fight them.
    if not i.in_guided:
        return ControlAction.RELEASE

    # 2. Proximity danger
    if i.reflex_danger or (i.estop_emergency and i.estop_recede):
        if i.ekf_ok and i.armed:
            return ControlAction.SET_BRAKE
        return ControlAction.STREAM_ZERO

    # 3. Confirmed target-lost emergency.
    if i.estop_emergency and not i.estop_recede:
        if i.ekf_ok and i.armed:
            return ControlAction.SET_LOITER
        return ControlAction.STREAM_ZERO

    # 4. Stale command or a brief-dropout hold
    if not i.command_fresh or i.estop_hold:
        if i.holding and i.ekf_ok:
            return ControlAction.HOLD_POSITION
        return ControlAction.STREAM_ZERO

    # 5. Nominal: stream the follow velocity.
    return ControlAction.STREAM_VELOCITY


def should_request(
    current_mode: str,
    want_mode: str,
    last_req_mode: str,
    last_req_s: float,
    now_s: float,
    min_interval_s: float = 0.5,
) -> bool:
    """Throttle flight-mode service requests"""
    if current_mode == want_mode:
        return False
    if last_req_mode == want_mode and (now_s - last_req_s) < min_interval_s:
        return False
    return True
