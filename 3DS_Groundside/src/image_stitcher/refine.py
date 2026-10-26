"""
Global refinement of photo poses ("bundle adjustment" for flat ground).

Placing photos one at a time lets small matching errors pile up along
chains of photos, so the map slowly drifts and overlapping photos stop lining
up. This re-solves every photo's pose at once so that all matched points
agree as well as possible.

Each pose is a similarity transform from image pixels to a shared frame:

    [a  -b  tx]
    [b   a  ty]

so a matched point p in photo i and q in photo j gives two equations that
are linear in the unknowns (a, b, tx, ty) of both photos:

    pose_i(p) - pose_j(q) = 0

Stacking every match gives one linear least-squares problem.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PointMatches:
    """
    The same ground points seen in photo `i` (points_i) and photo `j` (points_j).
    """

    i: int
    j: int
    points_i: np.ndarray
    points_j: np.ndarray


def refine_poses(
    poses: list[np.ndarray],
    matches: list[PointMatches],
    fixed: int = 0,
    max_points_per_pair: int = 100,
) -> list[np.ndarray]:
    """
    Returns poses that best agree with all matches.

    poses: 3x3 similarity transforms, image pixels -> shared frame
    matches: verified point matches between pairs of photos
    fixed: index of the photo whose pose stays put, pinning the frame
    max_points_per_pair: matches used per photo pair, keeps solving fast and
        stops pairs with many matches from dominating
    """
    count = len(poses)
    unknowns = 4 * count
    normal_matrix = np.zeros((unknowns, unknowns))

    for pair in matches:
        points_i, points_j = _subsample(pair, max_points_per_pair)
        # Rows: x then y equation for every point. Columns: (a, b, tx, ty)
        # of photo i, then of photo j.
        rows = np.zeros((2 * len(points_i), 8))
        rows[0::2, :4] = _x_equation(points_i)
        rows[1::2, :4] = _y_equation(points_i)
        rows[0::2, 4:] = -_x_equation(points_j)
        rows[1::2, 4:] = -_y_equation(points_j)
        columns = np.r_[4 * pair.i : 4 * pair.i + 4, 4 * pair.j : 4 * pair.j + 4]
        normal_matrix[np.ix_(columns, columns)] += rows.T @ rows

    # The equations only say how photos sit relative to each other, so one
    # photo's pose is held at its current value and the rest are solved for.
    pinned = np.arange(4 * fixed, 4 * fixed + 4)
    free = np.setdiff1d(np.arange(unknowns), pinned)
    pinned_values = _parameters(poses[fixed])
    right_side = -normal_matrix[np.ix_(free, pinned)] @ pinned_values
    free_values = np.linalg.lstsq(
        normal_matrix[np.ix_(free, free)], right_side, rcond=None
    )[0]

    solution = np.empty(unknowns)
    solution[pinned] = pinned_values
    solution[free] = free_values
    return [_pose(solution[4 * k : 4 * k + 4]) for k in range(count)]


def _x_equation(points: np.ndarray) -> np.ndarray:
    # x' = a * x - b * y + tx
    return np.column_stack(
        [points[:, 0], -points[:, 1], np.ones(len(points)), np.zeros(len(points))]
    )


def _y_equation(points: np.ndarray) -> np.ndarray:
    # y' = b * x + a * y + ty
    return np.column_stack(
        [points[:, 1], points[:, 0], np.zeros(len(points)), np.ones(len(points))]
    )


def _subsample(pair: PointMatches, limit: int) -> tuple[np.ndarray, np.ndarray]:
    points_i = np.asarray(pair.points_i, dtype=np.float64)
    points_j = np.asarray(pair.points_j, dtype=np.float64)
    if len(points_i) > limit:
        keep = np.random.default_rng(0).choice(len(points_i), limit, replace=False)
        points_i, points_j = points_i[keep], points_j[keep]
    return points_i, points_j


def _parameters(pose: np.ndarray) -> np.ndarray:
    return np.array([pose[0, 0], pose[1, 0], pose[0, 2], pose[1, 2]])


def _pose(parameters: np.ndarray) -> np.ndarray:
    a, b, tx, ty = parameters
    return np.array([[a, -b, tx], [b, a, ty], [0.0, 0.0, 1.0]])
