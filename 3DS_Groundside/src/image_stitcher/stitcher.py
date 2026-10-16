"""
Groundside orthomosaic stitcher.

Photos arrive in any order from any drone. For each new photo:

1. GPS picks the already-placed photos it probably overlaps.
2. XFeat finds the same ground points in the new photo and those neighbours,
   and RANSAC fits the exact shift, rotation and scale between them.
3. The photo joins the "cluster" of photos it lines up with (merging
   clusters when it bridges two). Photos that line up with nothing start a
   new cluster.

Inside a cluster, photos are positioned relative to each other purely from
image matches, which is pixel accurate. Every few photos, and before
drawing the map, all of a cluster's poses are re-solved together (see
refine.py) so small errors don't pile up along chains of photos.

Each cluster is placed on the map with a single best-fit rotation, scale and
shift against all of its photos' GPS positions, so individual GPS errors
average out instead of blurring the map. No heading or camera field of view
is needed.
"""

from __future__ import annotations

import logging
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from src.image_stitcher.canvas import Canvas, MapBounds
from src.image_stitcher.features import ImageFeatures, XFeatMatcher
from src.image_stitcher.files import write_map
from src.image_stitcher.geo import (
    FLIP_Y,
    apply,
    fit_similarity,
    gps_to_local,
    image_centre,
    scale_of,
    to_3x3,
)
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.refine import PointMatches, refine_poses

logger = logging.getLogger(__name__)


@dataclass
class StitcherConfig:
    """
    Tuning knobs. The defaults suit 640 x 480 photos with 50%+ overlap.
    """

    # Fewest RANSAC-verified matches needed to trust that two photos overlap.
    min_inliers: int = 30
    # How far (pixels) a match may be from the fitted transform and still count.
    ransac_threshold_px: float = 3.0
    # Most already-placed photos to try matching each new photo against.
    max_neighbours: int = 6
    # Worst GPS error expected (metres). Widens the neighbour search.
    gps_error_m: float = 5.0
    # Reject an image match that puts a photo this far (metres) from its GPS.
    max_gps_disagreement_m: float = 15.0
    # Two photos' scales should differ only by their altitude ratio; allowed
    # relative error before a match is rejected as wrong.
    max_scale_error: float = 0.25
    # A cluster's photos must spread at least this far (metres) before its
    # rotation and scale are fitted from GPS.
    min_anchor_spread_m: float = 5.0
    # Camera field of view (degrees), only used for photos that cannot be
    # matched to anything before any cluster has learned the real scale.
    fallback_fov_deg: float = 70.0
    # Re-solve all poses together every this many photos (0 = only when
    # rendering). Removes drift; cost grows with the number of photos.
    refine_every: int = 10
    # Output metres per pixel. None uses the photos' median resolution.
    resolution_m: float | None = None
    # Largest allowed map side in pixels, guards against corrupt positions.
    max_map_size_px: int = 20_000


@dataclass
class StitchedMap:
    """
    A rendered orthomosaic.

    rgb: H x W x 3 map, north up
    coverage: H x W bool, False where no photo covers the ground
    bounds: GPS of the map image's corners
    resolution_m: metres of ground per map pixel
    """

    rgb: np.ndarray
    coverage: np.ndarray
    bounds: MapBounds
    resolution_m: float
    image_count: int


@dataclass
class _Photo:
    message: ImageMessage
    features: ImageFeatures
    # GPS position in local metres.
    east: float
    north: float
    cluster: int = -1
    # Image pixels -> cluster frame (the pixel frame of the cluster's first photo).
    pose: np.ndarray = field(default_factory=lambda: np.eye(3))

    @property
    def centre_px(self) -> np.ndarray:
        height, width = self.message.img.shape[:2]
        return image_centre(width, height)


@dataclass
class _Match:
    """
    Verified matching points between a new photo and one placed photo.
    """

    other: _Photo
    points_new: np.ndarray
    # The same points in the other photo's pixels, and in its cluster frame.
    points_other: np.ndarray
    points_cluster: np.ndarray


@dataclass
class _Link:
    """
    Verified matching points between two placed photos, kept for refinement.
    """

    a: _Photo
    b: _Photo
    points_a: np.ndarray
    points_b: np.ndarray


class ImageStitcher:
    """
    Incrementally builds one orthomosaic from geotagged drone photos.
    """

    def __init__(
        self,
        config: StitcherConfig | None = None,
        matcher: XFeatMatcher | None = None,
    ) -> None:
        self._config = config or StitcherConfig()
        self._matcher = matcher or XFeatMatcher()
        self._photos: list[_Photo] = []
        self._links: list[_Link] = []
        # Clusters whose poses changed since they were last refined.
        self._unrefined: set[int] = set()
        self._reference: tuple[float, float] | None = None
        self._next_cluster = 0
        # Cluster -> anchored fit (or None). Cleared whenever photos are
        # added or clusters merge.
        self._fit_cache: dict[int, np.ndarray | None] = {}

    @property
    def image_count(self) -> int:
        return len(self._photos)

    @property
    def cluster_count(self) -> int:
        return len({photo.cluster for photo in self._photos})

    @property
    def reference(self) -> tuple[float, float] | None:
        """
        (lat, lon) of local (0, 0) metres: the first photo's position.
        """
        return self._reference

    def add(self, message: ImageMessage) -> bool:
        """
        Adds one photo to the map. Returns False if it was skipped.
        """
        if not message.has_valid_position():
            logger.warning("Skipping %r: no valid GPS position", message.image_id)
            return False
        if not message.has_valid_image():
            logger.warning("Skipping %r: image is not H x W x 3", message.image_id)
            return False

        if self._reference is None:
            self._reference = (message.lat, message.lon)
        east, north = gps_to_local(message.lat, message.lon, *self._reference)
        photo = _Photo(message, self._matcher.extract(message.img), east, north)

        self._place(photo, self._find_matches(photo))
        # Later photos only match against the upright features, so drop the
        # rotated copies to save memory.
        photo.features = ImageFeatures({0: photo.features.upright})
        self._photos.append(photo)
        self._unrefined.add(photo.cluster)
        self._fit_cache.clear()
        if (
            self._config.refine_every
            and self.image_count % self._config.refine_every == 0
        ):
            self.refine()
        logger.info(
            "Added %r from %r: %d photos in %d cluster(s)",
            message.image_id,
            message.drone_id,
            self.image_count,
            self.cluster_count,
        )
        return True

    def process_queue(self, queue: deque) -> int:
        """
        Adds every photo currently waiting in the queue. Returns how many
        were taken off the queue.
        """
        count = 0
        while True:
            try:
                message = queue.popleft()
            except IndexError:
                return count
            self.add(message)
            count += 1

    def run(
        self,
        queue: deque,
        stop: threading.Event,
        output_path: Path | None = None,
        save_every: int = 0,
        poll_interval_s: float = 0.1,
    ) -> None:
        """
        Keeps stitching photos from the queue until `stop` is set and the
        queue is empty.

        If output_path is given, the map is saved there every `save_every`
        photos (0 = only at the end) and once more when stopping.
        """
        while True:
            try:
                message = queue.popleft()
            except IndexError:
                if stop.is_set():
                    break
                stop.wait(poll_interval_s)
                continue
            added = self.add(message)
            if (
                added
                and output_path is not None
                and save_every
                and self.image_count % save_every == 0
            ):
                self.save(output_path)

        if output_path is not None and self.image_count > 0:
            self.save(output_path)

    def render(self) -> StitchedMap:
        """
        Draws every photo onto one north-up map.
        """
        if not self._photos:
            raise ValueError("no photos have been added")
        self.refine()
        resolution_m = self._config.resolution_m or float(
            np.median([self._metres_per_px(photo) for photo in self._photos])
        )
        canvas = Canvas(resolution_m, self._reference, self._config.max_map_size_px)
        for photo in self._photos:
            canvas.add(photo.message.img, self._image_to_local(photo))
        rgb, coverage = canvas.render()
        return StitchedMap(
            rgb, coverage, canvas.bounds(), resolution_m, self.image_count
        )

    def save(self, path: Path) -> None:
        """
        Renders the map and writes it as a PNG plus a JSON of its GPS corners.
        """
        write_map(path, self.render())

    def refine(self) -> None:
        """
        Re-solves the poses of every photo in each changed cluster at once,
        removing drift that builds up from placing photos one at a time.
        Runs automatically every `refine_every` photos and before rendering.
        """
        for cluster in self._unrefined:
            members = self._members(cluster)
            if len(members) < 2:
                continue
            index = {id(photo): k for k, photo in enumerate(members)}
            matches = [
                PointMatches(
                    index[id(link.a)], index[id(link.b)], link.points_a, link.points_b
                )
                for link in self._links
                if id(link.a) in index and id(link.b) in index
            ]
            poses = refine_poses([photo.pose for photo in members], matches)
            for photo, pose in zip(members, poses):
                photo.pose = pose
        self._unrefined.clear()
        self._fit_cache.clear()

    def placements(self) -> list[tuple[ImageMessage, np.ndarray]]:
        """
        Each photo's final placement, in the order added: (message, 3x3
        transform from its pixels to local metres east/north of `reference`).

        Returned per message rather than by image_id because different drones
        can name their photos the same way.
        """
        return [(photo.message, self._image_to_local(photo)) for photo in self._photos]

    def _find_matches(self, photo: _Photo) -> dict[int, list[_Match]]:
        """
        Matches the new photo against nearby placed photos, grouped by cluster.
        """
        matches: dict[int, list[_Match]] = {}
        for other in self._neighbours(photo):
            verified = self._verified_match(photo, other)
            if verified is None:
                continue
            points_new, points_other = verified
            matches.setdefault(other.cluster, []).append(
                _Match(other, points_new, points_other, apply(other.pose, points_other))
            )
        return matches

    def _verified_match(
        self, photo: _Photo, other: _Photo
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """
        The RANSAC-verified matching points between two photos, trying each
        quarter turn of the new photo (headings are unknown), or None if
        they don't convincingly overlap.
        """
        config = self._config
        best = None
        for features in photo.features.by_turn.values():
            points_new, points_other = self._matcher.match(
                features, other.features.upright
            )
            if len(points_new) < config.min_inliers:
                continue
            affine, inliers = cv2.estimateAffinePartial2D(
                points_new,
                points_other,
                method=cv2.RANSAC,
                ransacReprojThreshold=config.ransac_threshold_px,
            )
            if affine is None:
                continue
            inliers = inliers.ravel().astype(bool)
            if inliers.sum() < config.min_inliers:
                continue

            # A photo from higher up shows more ground per pixel, so its
            # pixels map to proportionally more pixels of the other photo.
            expected_scale = photo.message.alt / other.message.alt
            scale_error = abs(math.log(scale_of(to_3x3(affine)) / expected_scale))
            if scale_error > math.log(1 + config.max_scale_error):
                continue

            if best is None or inliers.sum() > len(best[0]):
                best = (points_new[inliers], points_other[inliers])
        return best

    def _neighbours(self, photo: _Photo) -> list[_Photo]:
        """
        Placed photos close enough by GPS that they might overlap, nearest first.
        """
        height, width = photo.message.img.shape[:2]
        footprint_diagonal = self._metres_per_px(photo) * math.hypot(width, height)
        radius = footprint_diagonal + 2 * self._config.gps_error_m

        def distance(other: _Photo) -> float:
            return math.hypot(other.east - photo.east, other.north - photo.north)

        nearby = [other for other in self._photos if distance(other) <= radius]
        nearby.sort(key=distance)
        return nearby[: self._config.max_neighbours]

    def _place(self, photo: _Photo, matches: dict[int, list[_Match]]) -> None:
        """
        Sets the photo's cluster and pose, merging clusters it connects.
        """
        # Try the clusters with the most supporting matches first.
        ranked = sorted(
            matches.items(),
            key=lambda item: -sum(len(match.points_new) for match in item[1]),
        )
        poses: dict[int, np.ndarray] = {}
        for cluster, cluster_matches in ranked:
            pose = self._estimate_pose(cluster_matches)
            if pose is None:
                continue
            if not self._agrees_with_gps(photo, cluster, pose):
                logger.info(
                    "Rejected match of %r to cluster %d: too far from its GPS",
                    photo.message.image_id,
                    cluster,
                )
                continue
            poses[cluster] = pose

        if not poses:
            photo.cluster = self._next_cluster
            photo.pose = np.eye(3)
            self._next_cluster += 1
            return

        target, *others = poses
        photo.cluster = target
        photo.pose = poses[target]
        for cluster in poses:
            for match in matches[cluster]:
                self._links.append(
                    _Link(photo, match.other, match.points_new, match.points_other)
                )
        for other in others:
            # other frame -> photo pixels -> target frame
            other_to_target = poses[target] @ np.linalg.inv(poses[other])
            for placed in self._photos:
                if placed.cluster == other:
                    placed.cluster = target
                    placed.pose = other_to_target @ placed.pose
            self._unrefined.discard(other)
            logger.info("Merged cluster %d into %d", other, target)

    def _estimate_pose(self, matches: list[_Match]) -> np.ndarray | None:
        """
        Fits the new photo's pixels -> cluster frame transform from all its
        matches into that cluster at once.
        """
        points_new = np.concatenate([match.points_new for match in matches])
        points_cluster = np.concatenate([match.points_cluster for match in matches])
        affine, inliers = cv2.estimateAffinePartial2D(
            points_new.astype(np.float32),
            points_cluster.astype(np.float32),
            method=cv2.RANSAC,
            ransacReprojThreshold=self._config.ransac_threshold_px,
        )
        if affine is None or inliers.sum() < self._config.min_inliers:
            return None
        return to_3x3(affine)

    def _agrees_with_gps(self, photo: _Photo, cluster: int, pose: np.ndarray) -> bool:
        """
        Guards against wrong matches (e.g. repetitive grass or roofs): once a
        cluster's place on the map is well known, a photo joining it must land
        near its GPS.

        A cluster's map position is only trusted once its photos spread over
        several times the GPS error; before that, a few bad GPS readings could
        misplace the cluster itself and wrongly reject good matches.
        """
        if _gps_spread(self._members(cluster)) < 4 * self._config.gps_error_m:
            return True
        frame_to_local, anchored = self._georeference(cluster)
        if not anchored:
            return True
        east, north = apply(frame_to_local @ pose, photo.centre_px)[0]
        error = math.hypot(east - photo.east, north - photo.north)
        return error <= self._config.max_gps_disagreement_m

    def _image_to_local(self, photo: _Photo) -> np.ndarray:
        frame_to_local, _ = self._georeference(photo.cluster)
        return frame_to_local @ photo.pose

    def _georeference(self, cluster: int) -> tuple[np.ndarray, bool]:
        """
        Transform from a cluster's frame to local metres, and whether it was
        fitted from GPS ("anchored") rather than guessed.

        Unanchored clusters (one photo, or all taken from about the same
        spot) get their position from GPS, their scale from what anchored
        clusters learned, and assume the image top faces north.
        """
        anchored_fit = self._anchored_fit(cluster)
        if anchored_fit is not None:
            return anchored_fit, True

        members = self._members(cluster)
        centres = np.array([apply(photo.pose, photo.centre_px)[0] for photo in members])
        gps = np.array([[photo.east, photo.north] for photo in members])
        metres_per_frame_px = float(
            np.median(
                [self._metres_per_px(photo) / scale_of(photo.pose) for photo in members]
            )
        )
        to_origin = np.eye(3)
        to_origin[:2, 2] = -centres.mean(axis=0)
        to_gps = np.eye(3)
        to_gps[:2, 2] = gps.mean(axis=0)
        scale = np.diag([metres_per_frame_px, metres_per_frame_px, 1.0])
        return to_gps @ scale @ FLIP_Y @ to_origin, False

    def _anchored_fit(self, cluster: int) -> np.ndarray | None:
        """
        Best-fit rotation, scale and shift from a cluster's frame to local
        metres, matching where its photo centres are in the frame to their GPS
        positions. None if the photos are too few or too close together.
        """
        if cluster in self._fit_cache:
            return self._fit_cache[cluster]

        members = self._members(cluster)
        centres = np.array([apply(photo.pose, photo.centre_px)[0] for photo in members])
        gps = np.array([[photo.east, photo.north] for photo in members])
        fit = None
        if (
            len(members) >= 2
            and _gps_spread(members) >= self._config.min_anchor_spread_m
        ):
            # Image y points down but north points up, so flip y first and
            # fit a reflection-free similarity.
            fit = fit_similarity(apply(FLIP_Y, centres), gps) @ FLIP_Y
        self._fit_cache[cluster] = fit
        return fit

    def _members(self, cluster: int) -> list[_Photo]:
        return [photo for photo in self._photos if photo.cluster == cluster]

    def _metres_per_px(self, photo: _Photo) -> float:
        """
        Estimated ground metres per pixel of a photo.

        Ground resolution is proportional to altitude, so the ratio learned
        from anchored clusters applies to any photo from the same camera.
        Before any cluster is anchored, the fallback field of view is used.
        """
        ratios = []
        for cluster in {placed.cluster for placed in self._photos}:
            fit = self._anchored_fit(cluster)
            if fit is None:
                continue
            ratios.extend(
                scale_of(fit) * scale_of(placed.pose) / placed.message.alt
                for placed in self._members(cluster)
            )
        if ratios:
            return float(np.median(ratios)) * photo.message.alt

        width = photo.message.img.shape[1]
        half_fov = math.radians(self._config.fallback_fov_deg) / 2
        return 2 * photo.message.alt * math.tan(half_fov) / width


def _gps_spread(photos: list[_Photo]) -> float:
    """
    Furthest any photo's GPS position is from the group's average, in metres.
    """
    gps = np.array([[photo.east, photo.north] for photo in photos])
    return float(np.linalg.norm(gps - gps.mean(axis=0), axis=1).max())
