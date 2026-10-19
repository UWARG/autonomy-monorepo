import math

import numpy as np

from src.image_stitcher.geo import apply
from src.image_stitcher.refine import PointMatches, refine_poses


def similarity(angle_deg, scale, tx, ty):
    angle = math.radians(angle_deg)
    a, b = scale * math.cos(angle), scale * math.sin(angle)
    return np.array([[a, -b, tx], [b, a, ty], [0, 0, 1]])


def test_refinement_removes_accumulated_drift():
    # A row of photos, each overlapping the next, with exact matches.
    rng = np.random.default_rng(0)
    truth = [similarity(3 * k, 1 + 0.01 * k, 300 * k, 20 * k) for k in range(6)]
    matches = []
    for k in range(5):
        ground = rng.random((50, 2)) * 200 + [300 * k + 250, 20 * k]
        points_k = apply(np.linalg.inv(truth[k]), ground)
        points_next = apply(np.linalg.inv(truth[k + 1]), ground)
        matches.append(PointMatches(k, k + 1, points_k, points_next))

    # Start from poses that drifted further the longer the chain gets.
    drifted = [truth[0]] + [
        similarity(2, 1.02, 5 * k, -4 * k) @ pose for k, pose in enumerate(truth[1:], 1)
    ]
    refined = refine_poses(drifted, matches, fixed=0)

    for pose, true_pose in zip(refined, truth):
        np.testing.assert_allclose(pose, true_pose, atol=1e-3)


def test_fixed_photo_does_not_move():
    pose_a = similarity(10, 2.0, 5, 6)
    pose_b = similarity(0, 1.0, 0, 0)
    points = np.random.default_rng(1).random((10, 2)) * 100
    matches = [PointMatches(0, 1, points, apply(pose_a, points))]
    refined = refine_poses([pose_a, pose_b], matches, fixed=0)
    np.testing.assert_allclose(refined[0], pose_a, atol=1e-6)
    # Photo 1 must map its points where photo 0 maps the same ground points.
    np.testing.assert_allclose(
        apply(refined[1], apply(pose_a, points)), apply(pose_a, points), atol=1e-4
    )
