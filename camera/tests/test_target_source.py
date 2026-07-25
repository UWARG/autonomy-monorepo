"""Gate 0 -- the swappable target source seam, clock frozen for determinism."""

from types import SimpleNamespace

import pytest

from src.target_source import (
    AbstractTargetSource,
    DepthAITrackletProvider,
    RealTargetSource,
    SimTargetSource,
    TargetObservation,
    TrackletPacket,
    calibrate_xy,
    calibrate_z,
    select_closest_tracked,
)


def _tracklet(x, y, z, status="TRACKED", track_id=1):
    return SimpleNamespace(
        id=track_id,
        status=status,
        spatialCoordinates=SimpleNamespace(x=x, y=y, z=z),
    )


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
    src = SimTargetSource(
        clock=clock, z_centre_mm=3000.0, z_amplitude_mm=800.0, x_amplitude_mm=900.0
    )
    assert src.initialize() is True

    obs = src.get_target()  # t == 0 relative to t0
    assert isinstance(obs, TargetObservation)
    assert obs.tracked is True
    assert abs(obs.z_mm - 3000.0) < 1e-9  # sin(0) -> centre depth
    assert abs(obs.x_mm) < 1e-9
    assert obs.timestamp == clock.t


def test_sim_target_moves_with_time():
    clock = FakeClock()
    src = SimTargetSource(
        clock=clock, z_centre_mm=3000.0, z_amplitude_mm=800.0, period_s=12.0
    )
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


def test_acquisition_rejects_person_beyond_validated_range():
    source = RealTargetSource(poll_fn=lambda: [_tracklet(0, 0, 3001, track_id=4)])
    assert not source.enable()
    assert source.locked_track_id is None


def test_active_owner_reports_out_of_range_and_can_briefly_reacquire():
    packets = iter(
        [
            _packet([_tracklet(0, 0, 2900, track_id=4)]),
            _packet([_tracklet(0, 0, 3100, track_id=4)], 11),
            _packet([_tracklet(0, 0, 2800, track_id=4)], 12),
        ]
    )
    source = RealTargetSource(poll_fn=lambda: next(packets))
    assert source.enable()
    assert source.get_target().within_validated_range
    assert not source.get_target().within_validated_range
    assert source.locked_track_id == 4
    assert source.get_target().within_validated_range


def test_real_target_source_emits_calibrated_observation():
    tracklets = [_tracklet(300, 0, 1000, status="TRACKED")]
    src = RealTargetSource(
        poll_fn=lambda: tracklets, tracked_status="TRACKED", clock=lambda: 7.0
    )
    assert src.initialize() is True
    assert src.enable() is True
    obs = src.get_target()
    assert isinstance(obs, TargetObservation)
    cal_z = calibrate_z(1000.0)
    cal_x, _ = calibrate_xy(300.0, 0.0, 1000.0, cal_z)
    assert abs(obs.z_mm - cal_z) < 1e-9
    assert abs(obs.x_mm - cal_x) < 1e-9
    assert obs.timestamp == 7.0
    assert obs.track_id == 1


def test_real_target_source_none_without_pipeline():
    src = RealTargetSource(poll_fn=None)
    assert src.initialize() is False
    assert src.get_target() is None


def _packet(tracklets, sequence=10, capture=5.0, received=5.1):
    return TrackletPacket(tuple(tracklets), capture, sequence, received)


def test_sticky_lock_ignores_closer_bystander_crossing():
    packets = iter(
        [
            _packet(
                [_tracklet(0, 0, 2000, track_id=4), _tracklet(0, 0, 3000, track_id=9)]
            ),
            _packet(
                [_tracklet(0, 0, 2500, track_id=4), _tracklet(0, 0, 500, track_id=9)],
                11,
            ),
        ]
    )
    source = RealTargetSource(poll_fn=lambda: next(packets))
    assert source.enable() is True
    assert source.locked_track_id == 4
    assert source.get_target().track_id == 4
    assert source.get_target().track_id == 4


def test_repeated_enable_cannot_replace_existing_owner():
    first = _packet([_tracklet(0, 0, 2000, track_id=4)])
    replacement = _packet([_tracklet(0, 0, 500, track_id=9)], 11)
    source = RealTargetSource(poll_fn=lambda: first)
    assert source.enable()
    assert not source.enable(replacement)
    assert source.locked_track_id == 4


def test_brief_loss_preserves_lock_and_same_id_reacquires():
    packets = iter(
        [
            _packet([_tracklet(0, 0, 2000, track_id=4)]),
            _packet([_tracklet(0, 0, 1900, status="LOST", track_id=4)], 11),
            _packet([_tracklet(0, 0, 1800, track_id=4)], 12),
        ]
    )
    source = RealTargetSource(poll_fn=lambda: next(packets))
    assert source.enable()
    assert source.get_target() is not None
    assert source.get_target() is None
    assert source.locked_track_id == 4
    assert source.get_target().track_id == 4


def test_reset_requires_new_explicit_enable():
    packet = _packet([_tracklet(0, 0, 2000, track_id=4)])
    source = RealTargetSource(poll_fn=lambda: packet)
    assert source.enable()
    source.reset_target()
    assert source.locked_track_id is None
    assert source.get_target() is None


def test_capture_metadata_passes_through():
    packet = _packet([_tracklet(1, 2, 1000, track_id=7)], sequence=42, capture=123.5)
    source = RealTargetSource(poll_fn=lambda: packet)
    assert source.enable()
    observation = source.get_target()
    assert observation.track_id == 7
    assert observation.sequence_num == 42
    assert observation.capture_time_s == 123.5


def test_depthai_provider_uses_packet_timestamp_and_sequence():
    class Queue:
        def tryGet(self):
            return SimpleNamespace(
                tracklets=[_tracklet(0, 0, 1000)],
                getTimestamp=lambda: SimpleNamespace(total_seconds=lambda: 11.25),
                getSequenceNum=lambda: 99,
            )

    packet = DepthAITrackletProvider(Queue(), clock=lambda: 11.3).poll()
    assert packet.capture_time_s == 11.25
    assert packet.received_time_s == 11.3
    assert packet.sequence_num == 99
    assert packet.detector_confirmed
    assert packet.detector_sequence_num == 99


def test_depthai_provider_marks_only_matching_detector_sequence_confirmed():
    class Queue:
        def __init__(self, values):
            self.values = iter(values)

        def tryGet(self):
            return next(self.values, None)

    def packet(sequence, timestamp):
        return SimpleNamespace(
            tracklets=[_tracklet(0, 0, 1000)],
            getTimestamp=lambda: SimpleNamespace(total_seconds=lambda: timestamp),
            getSequenceNum=lambda: sequence,
        )

    # Metadata may arrive ahead of the tracker queue; frame 11 must still
    # refer to the last detector at or before it, never future frame 12.
    detector_queue = Queue([packet(10, 1.0), packet(12, 1.1), None, None])
    tracker_queue = Queue([packet(10, 1.0), packet(11, 1.05)])
    provider = DepthAITrackletProvider(
        tracker_queue,
        detector_queue=detector_queue,
        clock=lambda: 2.0,
    )
    confirmed = provider.poll()
    propagated = provider.poll()
    assert confirmed.detector_confirmed
    assert confirmed.detector_sequence_num == 10
    assert not propagated.detector_confirmed
    assert propagated.detector_sequence_num == 10
    assert propagated.detector_capture_time_s == 1.0
