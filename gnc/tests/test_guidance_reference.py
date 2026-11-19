import math

import pytest

from guidance import GuidanceLimits, ReferenceTrajectory, Viewpoint, build_reference


def straight_reference() -> ReferenceTrajectory:
    return build_reference(
        [
            Viewpoint((0.0, 0.0, 10.0)),
            Viewpoint((30.0, 0.0, 10.0), capture=True, look_at_m=(30.0, 0.0, 0.0)),
            Viewpoint((60.0, 0.0, 10.0)),
        ],
        GuidanceLimits(
            max_velocity_mps=5.0,
            max_acceleration_mps2=1.5,
            capture_speed_mps=0.5,
            sample_period_s=0.05,
        ),
    )


def test_reference_respects_limits_and_endpoints():
    reference = straight_reference()
    assert reference.speed_mps[0] == pytest.approx(0.0)
    assert reference.speed_mps[-1] == pytest.approx(0.0)
    assert max(reference.speed_mps) <= 5.0 + 1e-6
    assert max(
        math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        for a in reference.acceleration_mps2
    ) <= 1.5 + 1e-6
    assert reference.duration_s > 0.0


def test_reference_passes_through_viewpoints():
    reference = straight_reference()
    for point in ((0.0, 0.0, 10.0), (30.0, 0.0, 10.0), (60.0, 0.0, 10.0)):
        assert min(
            math.dist(sample, point) for sample in reference.position_m
        ) < 0.1


def test_capture_has_slow_down_speed():
    reference = straight_reference()
    capture = reference.captures[0]
    assert capture.index == 1
    assert reference.speed_mps[capture.sample_index] <= 0.5 + 1e-6


def test_aim_target_advances_to_next_look_at():
    reference = build_reference(
        [
            Viewpoint((0.0, 0.0, 10.0), look_at_m=(0.0, 0.0, 0.0)),
            Viewpoint((30.0, 0.0, 10.0)),
            Viewpoint((60.0, 0.0, 10.0), look_at_m=(60.0, 0.0, 0.0)),
        ],
        GuidanceLimits(sample_period_s=0.05),
    )
    assert reference.aim_targets_m[0] == (0.0, 0.0, 0.0)
    assert reference.aim_targets_m[-1] == (60.0, 0.0, 0.0)


def test_corner_deviation_limit_is_enforced():
    viewpoints = [
        Viewpoint((0.0, 0.0, 10.0)),
        Viewpoint((20.0, 0.0, 10.0)),
        Viewpoint((20.0, 10.0, 10.0)),
        Viewpoint((0.0, 10.0, 10.0)),
    ]
    with pytest.raises(ValueError, match="deviates"):
        build_reference(viewpoints, GuidanceLimits(max_deviation_m=0.5))


def test_reference_rejects_too_few_or_duplicate_viewpoints():
    with pytest.raises(ValueError, match="at least two"):
        build_reference([Viewpoint((0.0, 0.0, 10.0))], GuidanceLimits())
    with pytest.raises(ValueError, match="distinct"):
        build_reference(
            [Viewpoint((0.0, 0.0, 10.0)), Viewpoint((0.0, 0.0, 10.0))],
            GuidanceLimits(),
        )


def test_capture_requires_look_at():
    with pytest.raises(ValueError, match="look_at_m"):
        Viewpoint((0.0, 0.0, 10.0), capture=True)


def test_capture_speed_cannot_exceed_max_velocity():
    with pytest.raises(ValueError, match="capture_speed_mps"):
        build_reference(
            [
                Viewpoint((0.0, 0.0, 10.0)),
                Viewpoint(
                    (30.0, 0.0, 10.0),
                    capture=True,
                    look_at_m=(30.0, 0.0, 0.0),
                    capture_speed_mps=6.0,
                ),
            ],
            GuidanceLimits(max_velocity_mps=5.0),
        )


def test_capture_at_first_and_last_viewpoint():
    reference = build_reference(
        [
            Viewpoint((0.0, 0.0, 10.0), capture=True, look_at_m=(0.0, 0.0, 0.0)),
            Viewpoint((30.0, 0.0, 10.0)),
            Viewpoint((60.0, 0.0, 10.0), capture=True, look_at_m=(60.0, 0.0, 0.0)),
        ],
        GuidanceLimits(sample_period_s=0.05),
    )
    assert [capture.index for capture in reference.captures] == [0, 2]
    assert reference.speed_mps[0] == pytest.approx(0.0)
    assert reference.speed_mps[-1] == pytest.approx(0.0)


def test_reference_without_captures_is_valid():
    reference = build_reference(
        [Viewpoint((0.0, 0.0, 10.0)), Viewpoint((30.0, 0.0, 10.0))],
        GuidanceLimits(),
    )
    assert reference.captures == []
    assert reference.duration_s > 0.0
