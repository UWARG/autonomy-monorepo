"""Body-frame velocity command -> MAVROS TwistStamped (body-FLU) components"""

from dataclasses import dataclass

@dataclass(frozen=True)
class TwistComponents:
    linear_x: float  # forward
    linear_y: float  # left
    linear_z: float  # up
    angular_z: float  # yaw rate, CCW positive


def body_velocity_to_flu(
    v_forward: float, v_right: float, v_down: float, yaw_rate: float
) -> TwistComponents:
    """Map FRD body velocity (+ NED yaw rate) to FLU TwistStamped components."""
    return TwistComponents(
        linear_x=v_forward,
        linear_y=-v_right,  # right (FRD) -> left (FLU)
        linear_z=-v_down,  # down (FRD) -> up (FLU)
        angular_z=-yaw_rate,  # clockwise (NED) -> counter-clockwise (ENU)
    )


def slew(previous: float, target: float, dv_max: float) -> float:
    """Acceleration-only slew for one commanded-velocity axis"""
    speeding_up = abs(target) > abs(previous) and (
        previous == 0.0 or (target >= 0.0) == (previous >= 0.0)
    )
    if not speeding_up:
        return target
    step = dv_max if target > previous else -dv_max
    if abs(target - previous) <= dv_max:
        return target
    return previous + step
