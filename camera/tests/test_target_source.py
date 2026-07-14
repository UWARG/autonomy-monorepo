"""Gate 0 -- the swappable target source seam, clock frozen for determinism."""

from types import SimpleNamespace

import pytest

from src.target_source import (
    AbstractTargetSource,
    RealTargetSource,
    SimTargetSource,
    TargetObservation,
    calibrate_xy,
    calibrate_z,
    select_closest_tracked,
)


def _tracklet(x, y, z, status="TRACKED"):
    return SimpleNamespace(status=status, spatialCoordinates=SimpleNamespace(x=x, y=y, z=z))


class FakeClock:
    def __init__(self, t: float = 100.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def test_abstract_source_cannot_be_instantiated():
    with pytest.raises(TypeError):
        AbstractTargetSource()  # type: ignore[abstract]


def test_sim_target_initialises_and_centres_at_t0():
    clock = FakeClock()
    src = SimTargetSource(clock=clock, z_centre_mm=3000.0, z_amplitude_mm=800.0, x_amplitude_mm=900.0)
    assert src.initialize() is True

    obs = src.get_target()  # t == 0 relative to t0
    assert isinstance(obs, TargetObservation)
    assert obs.tracked is True
    assert abs(obs.z_mm - 3000.0) < 1e-9  # sin(0) -> centre depth
    assert abs(obs.x_mm) < 1e-9
    assert obs.timestamp == clock.t


def test_sim_target_moves_with_time():
    clock = FakeClock()
    src = SimTargetSource(clock=clock, z_centre_mm=3000.0, z_amplitude_mm=800.0, period_s=12.0)
    src.initialize()
    clock.t += 3.0  # quarter period -> w*t = pi/2 -> sin = 1
    obs = src.get_target()
    assert abs(obs.z_mm - 3800.0) < 1e-6  # centre + amplitude


def test_start_delegates_to_initialize():
    src = SimTargetSource(clock=FakeClock())
    assert src.start() is True


# --- OAK-D calibration + RealTargetSource (Gap 3d) ---------------------------

def test_calibrate_z_at_and_beyond_anchors():
    assert calibrate_z(527.0) == 527.0 - 27.5
    assert calibrate_z(400.0) == 400.0 - 27.5  # below first anchor: flat offset
    assert calibrate_z(3000.0) == 3000.0  # beyond 2200: trust the device
    # midpoint between (1075, 75.1) and (1573, 73.2): offset ~ 74.15
    mid = calibrate_z((1075.0 + 1573.0) / 2)
    assert abs(mid - (((1075.0 + 1573.0) / 2) - 74.15)) < 0.2


def test_calibrate_xy_scales_by_depth_ratio():
    cal_x, cal_y = calibrate_xy(300.0, -100.0, 1000.0, 900.0)
    assert abs(cal_x - 270.0) < 1e-9  # 300 * 0.9
    assert abs(cal_y + 90.0) < 1e-9


def test_calibrate_xy_passes_through_on_invalid_depth():
    assert calibrate_xy(300.0, -100.0, 0.0, 0.0) == (300.0, -100.0)


def test_select_closest_tracked_picks_min_z_tracked():
    tracklets = [
        _tracklet(0, 0, 4000, status="TRACKED"),
        _tracklet(100, 0, 2000, status="TRACKED"),  # closest
        _tracklet(0, 0, 1000, status="LOST"),  # ignored
    ]
    chosen = select_closest_tracked(tracklets, "TRACKED")
    assert chosen.spatialCoordinates.z == 2000


def test_select_closest_tracked_none_when_no_tracked():
    assert select_closest_tracked([_tracklet(0, 0, 1, status="NEW")], "TRACKED") is None


def test_real_target_source_emits_calibrated_observation():
    tracklets = [_tracklet(300, 0, 1000, status="TRACKED")]
    src = RealTargetSource(poll_fn=lambda: tracklets, tracked_status="TRACKED", clock=lambda: 7.0)
    assert src.initialize() is True
    obs = src.get_target()
    assert isinstance(obs, TargetObservation)
    cal_z = calibrate_z(1000.0)
    cal_x, _ = calibrate_xy(300.0, 0.0, 1000.0, cal_z)
    assert abs(obs.z_mm - cal_z) < 1e-9
    assert abs(obs.x_mm - cal_x) < 1e-9
    assert obs.timestamp == 7.0


def test_real_target_source_none_without_pipeline():
    src = RealTargetSource(poll_fn=None)
    assert src.initialize() is False
    assert src.get_target() is None
