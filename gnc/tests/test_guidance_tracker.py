import math

import pytest

from guidance import (
    GimbalModel,
    GimbalState,
    GuidanceLimits,
    GuidanceTracker,
    TrackerConfig,
    TrackerStatus,
    Viewpoint,
    build_reference,
)


def build_line(capture: bool = False):
    viewpoints = [Viewpoint((0.0, 0.0, 10.0)), Viewpoint((60.0, 0.0, 10.0))]
    if capture:
        viewpoints = [
            Viewpoint((0.0, 0.0, 10.0)),
            Viewpoint((30.0, 0.0, 10.0), capture=True, look_at_m=(30.0, 0.0, 0.0)),
            Viewpoint((60.0, 0.0, 10.0)),
        ]
    limits = GuidanceLimits(
        max_velocity_mps=5.0,
        max_acceleration_mps2=1.5,
        capture_speed_mps=0.5,
        sample_period_s=0.05,
        capture_radius_m=1.0,
    )
    return build_reference(viewpoints, limits), limits


def simulate(reference, limits, start, dt=0.05, steps=3000, config=None):
    tracker = GuidanceTracker(
        reference, limits, GimbalModel(), config
    )
    tracker.reset(start)

    position = start
    velocity = (0.0, 0.0, 0.0)
    status = TrackerStatus.TRACKING
    max_speed = 0.0
    max_acceleration = 0.0
    for _ in range(steps):
        command = tracker.step(position, velocity, GimbalState(), dt)
        status = command.status
        acceleration = command.acceleration_mps2
        max_acceleration = max(
            max_acceleration, math.sqrt(sum(value * value for value in acceleration))
        )
        velocity = tuple(
            velocity[index] + acceleration[index] * dt for index in range(3)
        )
        max_speed = max(max_speed, math.sqrt(sum(value * value for value in velocity)))
        position = tuple(
            position[index]
            + velocity[index] * dt
            + 0.5 * acceleration[index] * dt**2
            for index in range(3)
        )
        if status is TrackerStatus.COMPLETE:
            break
    return position, status, max_speed, max_acceleration


@pytest.mark.parametrize("dt", [0.05, 0.02])
def test_tracker_follows_straight_line(dt):
    reference, limits = build_line()
    position, status, max_speed, max_acceleration = simulate(
        reference, limits, (1.0, 1.0, 10.0), dt
    )
    assert status is TrackerStatus.COMPLETE
    assert math.dist(position, reference.final_position_m) < 1.0
    assert max_speed <= limits.max_velocity_mps + 1e-6
    assert max_acceleration <= limits.max_acceleration_mps2 + 1e-6


def test_tracker_follows_curved_path():
    limits = GuidanceLimits(sample_period_s=0.05)
    reference = build_reference(
        [
            Viewpoint((0.0, 0.0, 10.0)),
            Viewpoint((20.0, 10.0, 10.0)),
            Viewpoint((40.0, 0.0, 10.0)),
        ],
        limits,
    )
    position, status, _, _ = simulate(reference, limits, (0.0, 0.0, 10.0))
    assert status is TrackerStatus.COMPLETE
    assert math.dist(position, reference.final_position_m) < 1.0


def test_tracker_follows_vertical_path():
    limits = GuidanceLimits(sample_period_s=0.05)
    reference = build_reference(
        [Viewpoint((0.0, 0.0, 5.0)), Viewpoint((0.0, 0.0, 25.0))],
        limits,
    )
    position, status, max_speed, max_acceleration = simulate(
        reference, limits, (0.0, 0.0, 5.0), steps=2000
    )
    assert status is TrackerStatus.COMPLETE
    assert position[2] == pytest.approx(25.0, abs=1.0)
    assert max_speed <= limits.max_velocity_mps + 1e-6
    assert max_acceleration <= limits.max_acceleration_mps2 + 1e-6


def test_capture_waits_for_settle_time():
    reference, limits = build_line(capture=True)
    gimbal = GimbalModel(settle_time_s=0.5)
    tracker = GuidanceTracker(
        reference,
        limits,
        gimbal,
        TrackerConfig(max_aircraft_yaw_rate_radps=math.pi),
    )
    capture = reference.captures[0]
    tracker.reset(capture.position_m)

    far = GimbalState(yaw_rad=0.0, pitch_rad=0.0)
    for _ in range(10):
        command = tracker.step(capture.position_m, (0.0, 0.0, 0.0), far, 0.1)
        assert not command.capture_ready
        assert command.status is TrackerStatus.TRACKING

    command = tracker.step(capture.position_m, (0.0, 0.0, 0.0), far, 0.1)
    settled = GimbalState(
        yaw_rad=command.gimbal_yaw_rad, pitch_rad=command.gimbal_pitch_rad
    )
    for _ in range(4):
        command = tracker.step(capture.position_m, (0.0, 0.0, 0.0), settled, 0.1)
        assert not command.capture_ready

    command = tracker.step(capture.position_m, (0.0, 0.0, 0.0), settled, 0.1)
    assert command.capture_ready
    assert command.status is TrackerStatus.CAPTURE_READY
    assert command.capture_index == capture.index
    assert tracker.mark_captured(capture.index)


def test_gimbal_can_capture():
    reference, limits = build_line(capture=True)
    gimbal = GimbalModel(settle_time_s=0.5)
    tracker = GuidanceTracker(
        reference,
        limits,
        gimbal,
        TrackerConfig(max_aircraft_yaw_rate_radps=100.0),
    )
    capture = reference.captures[0]
    tracker.reset(capture.position_m)

    command = tracker.step(capture.position_m, (0.0, 0.0, 0.0), GimbalState(), 0.1)
    settled = GimbalState(
        yaw_rad=command.gimbal_yaw_rad,
        pitch_rad=command.gimbal_pitch_rad,
        roll_rad=command.gimbal_roll_rad,
    )
    for _ in range(5):
        command = tracker.step(capture.position_m, (0.0, 0.0, 0.0), settled, 0.1)
    assert command.capture_ready
    assert command.capture_index == capture.index


def test_capture_is_reported_missed_when_passed():
    reference, limits = build_line(capture=True)
    tracker = GuidanceTracker(reference, limits, GimbalModel())
    tracker.reset((40.0, 0.0, 10.0))
    command = tracker.step((40.0, 0.0, 10.0), (0.0, 0.0, 0.0), GimbalState(), 0.1)
    assert command.status is TrackerStatus.CAPTURE_MISSED
    assert tracker.mark_captured(reference.captures[0].index) is False


def test_capture_missed_then_next_capture_ready():
    limits = GuidanceLimits(
        max_velocity_mps=5.0,
        max_acceleration_mps2=1.5,
        capture_speed_mps=0.5,
        sample_period_s=0.05,
        capture_radius_m=1.0,
    )
    reference = build_reference(
        [
            Viewpoint((0.0, 0.0, 10.0)),
            Viewpoint((20.0, 0.0, 10.0), capture=True, look_at_m=(20.0, 0.0, 0.0)),
            Viewpoint((40.0, 0.0, 10.0), capture=True, look_at_m=(40.0, 0.0, 0.0)),
            Viewpoint((60.0, 0.0, 10.0)),
        ],
        limits,
    )
    tracker = GuidanceTracker(
        reference,
        limits,
        GimbalModel(),
        TrackerConfig(
            projection_window_samples=200, max_aircraft_yaw_rate_radps=100.0
        ),
    )
    tracker.reset((25.0, 0.0, 10.0))
    command = tracker.step((25.0, 0.0, 10.0), (0.0, 0.0, 0.0), GimbalState(), 0.1)
    assert command.status is TrackerStatus.CAPTURE_MISSED
    assert command.capture_index == 1

    command = tracker.step((40.0, 0.0, 10.0), (0.0, 0.0, 0.0), GimbalState(), 0.1)
    settled = GimbalState(
        yaw_rad=command.gimbal_yaw_rad,
        pitch_rad=command.gimbal_pitch_rad,
        roll_rad=command.gimbal_roll_rad,
    )
    for _ in range(8):
        command = tracker.step((40.0, 0.0, 10.0), (0.0, 0.0, 0.0), settled, 0.1)
    assert command.capture_ready
    assert command.capture_index == 2


def test_tracker_reports_complete_at_final_position():
    reference, limits = build_line()
    tracker = GuidanceTracker(reference, limits, GimbalModel())
    tracker.reset(reference.final_position_m)
    command = tracker.step(
        reference.final_position_m, (0.0, 0.0, 0.0), GimbalState(), 0.1
    )
    assert command.status is TrackerStatus.COMPLETE


def test_tracker_rejects_bad_inputs():
    reference, limits = build_line()
    tracker = GuidanceTracker(reference, limits, GimbalModel())
    tracker.reset((0.0, 0.0, 10.0))
    with pytest.raises(ValueError, match="finite"):
        tracker.step((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0), GimbalState(), 0.1)
    with pytest.raises(ValueError, match="dt_s"):
        tracker.step((0.0, 0.0, 10.0), (0.0, 0.0, 0.0), GimbalState(), 0.0)
    with pytest.raises(ValueError, match="aircraft_yaw_rate_radps"):
        tracker.step(
            (0.0, 0.0, 10.0),
            (0.0, 0.0, 0.0),
            GimbalState(),
            0.1,
            math.nan,
        )


def test_tracker_limits_jerk():
    limits = GuidanceLimits(max_jerk_mps3=1.0, sample_period_s=0.05)
    reference = build_reference(
        [Viewpoint((0.0, 0.0, 10.0)), Viewpoint((60.0, 0.0, 10.0))], limits
    )
    tracker = GuidanceTracker(reference, limits, GimbalModel())
    tracker.reset((0.0, 0.0, 10.0))

    position = (0.0, 0.0, 10.0)
    velocity = (0.0, 0.0, 0.0)
    previous = (0.0, 0.0, 0.0)
    for _ in range(100):
        command = tracker.step(position, velocity, GimbalState(), 0.05)
        delta = math.sqrt(
            sum((command.acceleration_mps2[i] - previous[i]) ** 2 for i in range(3))
        )
        assert delta <= 1.0 * 0.05 + 1e-6
        previous = command.acceleration_mps2
        velocity = tuple(
            velocity[i] + command.acceleration_mps2[i] * 0.05 for i in range(3)
        )
        position = tuple(
            position[i]
            + velocity[i] * 0.05
            + 0.5 * command.acceleration_mps2[i] * 0.05**2
            for i in range(3)
        )


def test_aircraft_yaw_rate_is_limited():
    limits = GuidanceLimits(sample_period_s=0.05)
    reference = build_reference(
        [Viewpoint((0.0, 0.0, 10.0)), Viewpoint((0.0, 60.0, 10.0))], limits
    )
    tracker = GuidanceTracker(
        reference,
        limits,
        GimbalModel(),
        TrackerConfig(max_aircraft_yaw_rate_radps=0.5),
    )
    tracker.reset((0.0, 0.0, 10.0), (1.0, 0.0, 0.0))
    command = tracker.step((0.0, 0.0, 10.0), (0.0, 0.0, 0.0), GimbalState(), 0.1)
    assert command.yaw_rad == pytest.approx(math.pi / 2.0 - 0.05)
