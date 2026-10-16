"""
Finds the same points in two overlapping photos using the XFeat feature
matcher (https://github.com/verlab/accelerated_features).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

from src.image_stitcher.third_party.xfeat import XFeat

# XFeat is not rotation invariant: it stops recognising points once an image
# is turned more than roughly 45 degrees. Drones on a lawn mower path turn
# 180 degrees every row, so features are extracted from the image rotated by
# each quarter turn, and matching tries all of them.
QUARTER_TURNS = (0, 1, 2, 3)


@dataclass
class Features:
    """
    Keypoints found in one image and a descriptor for each.

    keypoints: (N, 2) pixel coordinates (x, y) in the original, unrotated image
    descriptors: (N, 64) unit vectors; similar points have similar vectors
    """

    keypoints: np.ndarray
    descriptors: torch.Tensor

    def __len__(self) -> int:
        return len(self.keypoints)


@dataclass
class ImageFeatures:
    """
    Features of one image, extracted once per quarter turn.

    by_turn[k] came from the image rotated k * 90 degrees counterclockwise.
    """

    by_turn: dict[int, Features]

    @property
    def upright(self) -> Features:
        return self.by_turn[0]


class XFeatMatcher:
    """
    Extracts XFeat features once per image, then matches any two images'
    features with mutual nearest neighbours.

    Runs on the GPU if PyTorch can see one, otherwise on the CPU.
    """

    def __init__(self, top_k: int = 2048, min_cosine_similarity: float = -1.0) -> None:
        """
        top_k: most keypoints to keep per image
        min_cosine_similarity: drop matches whose descriptors are less similar
            than this (-1 keeps every mutual match and leaves filtering to
            RANSAC)
        """
        self._xfeat = XFeat(top_k=top_k)
        self._min_cosine_similarity = min_cosine_similarity

    def extract(self, rgb: np.ndarray) -> ImageFeatures:
        """
        Finds keypoints in an H x W x 3 RGB image at every quarter turn.
        """
        height, width = rgb.shape[:2]
        by_turn = {}
        for turns in QUARTER_TURNS:
            rotated = np.ascontiguousarray(np.rot90(rgb, turns))
            features = self._extract_one(rotated)
            features.keypoints = unrotate_points(
                features.keypoints, turns, width, height
            )
            by_turn[turns] = features
        return ImageFeatures(by_turn)

    def match(self, a: Features, b: Features) -> tuple[np.ndarray, np.ndarray]:
        """
        Pairs up keypoints that look the same in both images.

        Returns (points_a, points_b), two (M, 2) arrays where row i of each is
        the same ground point seen in image a and image b. Some pairs will be
        wrong; callers should filter them with RANSAC.
        """
        if len(a) == 0 or len(b) == 0:
            empty = np.zeros((0, 2), dtype=np.float32)
            return empty, empty
        with torch.inference_mode():
            index_a, index_b = self._xfeat.match(
                a.descriptors, b.descriptors, min_cossim=self._min_cosine_similarity
            )
        return (
            a.keypoints[index_a.cpu().numpy()],
            b.keypoints[index_b.cpu().numpy()],
        )

    def _extract_one(self, rgb: np.ndarray) -> Features:
        with torch.inference_mode():
            tensor = self._xfeat.parse_input(rgb)
            output = self._xfeat.detectAndCompute(tensor)[0]
        return Features(
            keypoints=output["keypoints"].cpu().numpy().astype(np.float32),
            descriptors=output["descriptors"],
        )


def unrotate_points(
    points: np.ndarray, turns: int, width: int, height: int
) -> np.ndarray:
    """
    Maps (x, y) points found in np.rot90(image, turns) back to the original
    width x height image.
    """
    x, y = points[:, 0], points[:, 1]
    turns %= 4
    if turns == 0:
        original = (x, y)
    elif turns == 1:
        original = (width - 1 - y, x)
    elif turns == 2:
        original = (width - 1 - x, height - 1 - y)
    else:
        original = (y, height - 1 - x)
    return np.stack(original, axis=1).astype(np.float32)
