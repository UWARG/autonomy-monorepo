"""Pure arm-readiness state used by the issue-96 SITL harness."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ArmReadinessSnapshot:
    """One dependency-free snapshot of flight-controller readiness."""

    gps_fix: int
    ekf_using_gps: bool
    home_position_seen: bool
    local_position_seen: bool
    prearm_ok: bool


def missing_arm_preconditions(
    snapshot: ArmReadinessSnapshot,
) -> tuple[str, ...]:
    """Return every prerequisite that is not currently satisfied."""

    missing: list[str] = []
    if snapshot.gps_fix < 3:
        missing.append("gps_3d_fix")
    if not snapshot.ekf_using_gps:
        missing.append("ekf_using_gps")
    if not snapshot.home_position_seen:
        missing.append("global_home_position")
    if not snapshot.local_position_seen:
        missing.append("local_position")
    if not snapshot.prearm_ok:
        missing.append("prearm_check")
    return tuple(missing)


class StableArmReadinessGate:
    """Require all arm prerequisites to remain true for a stable interval."""

    def __init__(self, stable_s: float) -> None:
        if stable_s < 0.0:
            raise ValueError("stable interval must be non-negative")
        self.stable_s = stable_s
        self.ready_since_s: float | None = None
        self.last_missing: tuple[str, ...] = ()

    def update(self, now_s: float, snapshot: ArmReadinessSnapshot) -> bool:
        """Update the gate and return whether readiness is now stable."""

        self.last_missing = missing_arm_preconditions(snapshot)
        if self.last_missing:
            self.ready_since_s = None
            return False

        if self.ready_since_s is None or now_s < self.ready_since_s:
            self.ready_since_s = now_s
        return now_s - self.ready_since_s >= self.stable_s
