"""
Finds the same points in two overlapping photos using the XFeat feature
matcher (https://github.com/verlab/accelerated_features).
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import torch

from src.image_stitcher.geo import apply, to_3x3

# XFeat isn't pip installable, so it is loaded from its GitHub repo with
# torch.hub. The first run downloads the code and weights (about 6 MB) into
# ~/.cache/torch/hub; later runs use that cache and need no internet.
# Pinned to one commit so upstream changes can't change our results.
XFEAT_COMMIT = "e92685f57f8318b18725c5c8c0bd28c7fe188d9a"
XFEAT_REPO = f"verlab/accelerated_features:{XFEAT_COMMIT}"
XFEAT_WEIGHTS_URL = (
    "https://github.com/verlab/accelerated_features/raw/"
    f"{XFEAT_COMMIT}/weights/xfeat.pt"
)

# XFeat is not rotation invariant: it stops recognising points once an image
# is turned more than roughly 45 degrees. Drones on a lawn mower path turn
# 180 degrees every row, so every photo is turned north-up with the drone's
# yaw before extracting features.
#
# Resampling a photo softens it, so a photo whose yaw is this close (degrees)
# to a quarter turn is just given that quarter turn, which loses nothing. Two
# such photos can end up twice this far apart, plus any compass error. In
# simulation 40 degrees apart dropped some pairs below the inlier limit, and
# 20 did not.
MAX_QUARTER_TURN_ERROR_DEG = 10.0

# Any other turn leaves empty corners, and the edge of the photo then looks
# like a strong feature. Keypoints this close (pixels) to it are dropped.
EDGE_MARGIN_PX = 16


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
        self._xfeat = load_xfeat(top_k)
        self._min_cosine_similarity = min_cosine_similarity

    def extract(self, rgb: np.ndarray, yaw_deg: float) -> Features:
        """
        Finds keypoints in an H x W x 3 RGB image whose top faces yaw_deg
        clockwise from north. Keypoints are returned in the image's own pixels.
        """
        height, width = rgb.shape[:2]
        turns = round(yaw_deg / 90)
        if abs(yaw_deg - 90 * turns) <= MAX_QUARTER_TURN_ERROR_DEG:
            north_up, north_up_to_image = quarter_turn(rgb, turns)
            margin_px = 0
        else:
            north_up, north_up_to_image = rotate_north_up(rgb, yaw_deg)
            margin_px = EDGE_MARGIN_PX
        found = self._extract_one(north_up)
        keypoints = apply(north_up_to_image, found.keypoints).astype(np.float32)
        keep = inside_photo(keypoints, width, height, margin_px)
        return Features(
            keypoints=keypoints[keep],
            descriptors=found.descriptors[
                torch.from_numpy(keep).to(found.descriptors.device)
            ],
        )

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


def load_xfeat(top_k: int) -> torch.nn.Module:
    """
    Loads the pretrained XFeat model from the pinned upstream commit.
    """
    # skip_validation: the commit hash can't change, so there's no need to ask
    # the GitHub API (which is rate limited) whether it exists.
    xfeat = torch.hub.load(
        XFEAT_REPO,
        "XFeat",
        pretrained=False,
        top_k=top_k,
        trust_repo=True,
        skip_validation=True,
    )
    weights = torch.hub.load_state_dict_from_url(
        XFEAT_WEIGHTS_URL,
        map_location=xfeat.dev,
        progress=False,
        weights_only=True,
    )
    xfeat.net.load_state_dict(weights)
    return xfeat


def quarter_turn(rgb: np.ndarray, turns: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Turns an image clockwise by this many quarter turns, losslessly.

    Returns the turned image and the 3 x 3 transform from its pixels back to
    the original image's pixels.
    """
    height, width = rgb.shape[:2]
    last_x, last_y = width - 1, height - 1
    to_original = {
        0: [[1, 0, 0], [0, 1, 0]],
        1: [[0, 1, 0], [-1, 0, last_y]],
        2: [[-1, 0, last_x], [0, -1, last_y]],
        3: [[0, -1, last_x], [1, 0, 0]],
    }[turns % 4]
    # np.rot90 turns counterclockwise.
    turned = np.ascontiguousarray(np.rot90(rgb, -turns % 4))
    return turned, to_3x3(np.array(to_original, dtype=np.float64))


def rotate_north_up(rgb: np.ndarray, yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Turns an image whose top faces yaw_deg so that its top faces north, on a
    canvas big enough to hold all of it.

    Returns the turned image and the 3 x 3 transform from its pixels back to
    the original image's pixels.
    """
    height, width = rgb.shape[:2]
    centre = ((width - 1) / 2, (height - 1) / 2)
    # North is yaw_deg counterclockwise of the image's top, so the image turns
    # clockwise by yaw_deg. OpenCV's positive angles are counterclockwise.
    rotation = cv2.getRotationMatrix2D(centre, -yaw_deg, 1.0)
    cos_abs, sin_abs = abs(rotation[0, 0]), abs(rotation[0, 1])
    new_width = int(np.ceil(width * cos_abs + height * sin_abs))
    new_height = int(np.ceil(width * sin_abs + height * cos_abs))
    rotation[0, 2] += (new_width - 1) / 2 - centre[0]
    rotation[1, 2] += (new_height - 1) / 2 - centre[1]
    turned = cv2.warpAffine(rgb, rotation, (new_width, new_height))
    return turned, to_3x3(cv2.invertAffineTransform(rotation))


def inside_photo(
    points: np.ndarray, width: int, height: int, margin_px: float
) -> np.ndarray:
    """
    Boolean mask of the (N, 2) pixel points at least margin_px inside a
    width x height image.
    """
    x, y = points[:, 0], points[:, 1]
    return (
        (x >= margin_px)
        & (x <= width - 1 - margin_px)
        & (y >= margin_px)
        & (y <= height - 1 - margin_px)
    )
