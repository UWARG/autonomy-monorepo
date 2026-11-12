"""
Groundside orthomosaic stitcher.

Photos arrive in any order from any drone. Unusable ones (blurry, badly
exposed, tilted; see quality.py) are rejected and never placed. The rest are
undistorted with the camera calibration (camera.py), which also gives their
ground resolution from their altitude. Then:

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
average out instead of blurring the map. Yaw only turns photos north-up for
matching and orients clusters too small to fit, so compass error can't bend
the map.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from src.image_stitcher.camera import Camera
from src.image_stitcher.canvas import Canvas, MapBounds
from src.image_stitcher.features import Features, XFeatMatcher
from src.image_stitcher.files import next_map_path, write_map
from src.image_stitcher.geo import (
    FLIP_Y,
    apply,
    average_rotation,
    fit_similarity,
    gps_to_local,
    image_axes_to_local,
    scale_of,
    to_3x3,
)
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.quality import QualityLimits, rejection_reason
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
    # Worst GPS error expected (metres). Widens the neighbour search, and a
    # cluster's photos must spread 4 times this far before its place on the
    # map is trusted.
    gps_error_m: float = 5.0
    # Reject an image match that puts a photo this far (metres) from its GPS.
    max_gps_disagreement_m: float = 15.0
    # Two photos' scales should differ only by their altitude ratio; allowed
    # relative error before a match is rejected as wrong.
    max_scale_error: float = 0.25
    # A cluster's GPS positions must spread at least this far (metres) before
    # its rotation and scale are fitted from GPS. The photos themselves must
    # also spread 4 x gps_error_m, which is usually the stricter limit.
    min_anchor_spread_m: float = 5.0
    # Re-solve all poses together every this many photos (0 = only when
    # rendering). Removes drift; cost grows with the number of photos.
    refine_every: int = 10
    # Output metres per pixel. None uses the photos' median resolution.
    resolution_m: float | None = None
    # Largest allowed map side in pixels, guards against corrupt positions.
    max_map_size_px: int = 20_000
    # Reject photos whose GPS is further than this (metres) from the first
    # photo, i.e. GPS glitches. At 25 m this keeps the map within
    # max_map_size_px; lower flights need a smaller value.
    max_distance_m: float = 400.0
    # Limits for rejecting blurry, badly exposed or tilted photos.
    quality: QualityLimits = field(default_factory=QualityLimits)


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
    # Holds the undistorted image; all pixels below are in it.
    message: ImageMessage
    features: Features
    # GPS position in local metres.
    east: float
    north: float
    cluster: int = -1
    # Image pixels -> cluster frame (the pixel frame of the cluster's first photo).
    pose: np.ndarray = field(default_factory=lambda: np.eye(3))


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

    camera defaults to the calibration in camera_info.yaml.
    """

    def __init__(
        self,
        config: StitcherConfig | None = None,
        matcher: XFeatMatcher | None = None,
        camera: Camera | None = None,
    ) -> None:
        self._config = config or StitcherConfig()
        self._matcher = matcher or XFeatMatcher()
        self._camera = camera or Camera.load()
        self._photos: list[_Photo] = []
        self._rejections: list[tuple[str, str, str]] = []
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
    def rejections(self) -> list[tuple[str, str, str]]:
        """
        Every photo that was not used, in order: (drone_id, image_id, reason).
        """
        return list(self._rejections)

    @property
    def reference(self) -> tuple[float, float] | None:
        """
        (lat, lon) of local (0, 0) metres: the first photo's position.
        """
        return self._reference

    def add(self, message: ImageMessage) -> bool:
        """
        Adds one photo to the map. Returns False if it was rejected (see
        `rejections`).
        """
        reason = self._rejection_reason(message)
        if reason is not None:
            logger.warning(
                "Rejected %r from %r: %s", message.image_id, message.drone_id, reason
            )
            self._rejections.append((message.drone_id, message.image_id, reason))
            return False

        # The first photo becomes the map's reference, but only once its
        # features are extracted, so a photo that fails there never anchors
        # the map.
        reference = self._reference or (message.lat, message.lon)
        east, north = gps_to_local(message.lat, message.lon, *reference)
        # Keep only the undistorted image so memory doesn't double.
        message = dataclasses.replace(message, img=self._camera.undistort(message.img))
        features = self._matcher.extract(message.img, message.yaw_deg)
        photo = _Photo(message, features, east, north)
        self._reference = reference

        self._place(photo, self._find_matches(photo))
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
            self._add_safely(message)
            count += 1

    def run(
        self,
        queue: deque,
        stop: threading.Event,
        output_dir: Path | None = None,
        save_every: int = 0,
        poll_interval_s: float = 0.1,
    ) -> None:
        """
        Keeps stitching photos from the queue until `stop` is set and the
        queue is empty.

        If output_dir is given, the map is saved there every `save_every`
        photos (0 = only at the end) and once more when stopping. Each save is
        a new numbered file (map_001.png + map_001.json, map_002...), so the
        map's progress over the flight is kept.
        """
        saved_count = 0
        while True:
            try:
                message = queue.popleft()
            except IndexError:
                if stop.is_set():
                    break
                stop.wait(poll_interval_s)
                continue
            added = self._add_safely(message)
            if (
                added
                and output_dir is not None
                and save_every
                and self.image_count % save_every == 0
                and self._save_logging_errors(output_dir)
            ):
                saved_count = self.image_count

        # Skip the final save if the last periodic one already has every photo.
        if output_dir is not None and self.image_count > saved_count:
            self._save_logging_errors(output_dir)

    def _add_safely(self, message: ImageMessage) -> bool:
        # One malformed queue item must not stop the stitcher for the rest
        # of the flight: log it, record it as rejected and carry on.
        count_before = self.image_count
        try:
            return self.add(message)
        except Exception:
            drone_id = str(getattr(message, "drone_id", ""))
            image_id = str(getattr(message, "image_id", ""))
            logger.exception("Failed to add %r from %r", image_id, drone_id)
            if self.image_count > count_before:
                # It failed after being placed (e.g. while refining), so it
                # is on the map; don't also list it as rejected.
                return True
            self._rejections.append((drone_id, image_id, "error (see log)"))
            return False

    def _save_logging_errors(self, output_dir: Path) -> bool:
        # A failed save (full disk, map too large) is logged and the
        # stitcher keeps going; the next save tries again.
        try:
            self.save(next_map_path(output_dir))
            return True
        except Exception:
            logger.exception("Failed to save the map to %s", output_dir)
            return False

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

        The messages are copies holding the undistorted images, which is what
        the transforms apply to. Returned per message rather than by image_id
        because different drones can name their photos the same way.
        """
        return [(photo.message, self._image_to_local(photo)) for photo in self._photos]

    def _rejection_reason(self, message: ImageMessage) -> str | None:
        if not message.has_valid_position():
            return "no valid GPS position"
        if not message.has_valid_image():
            return "image is not 8-bit H x W x 3"
        if not self._camera.matches(message.img):
            height, width = message.img.shape[:2]
            return (
                f"image is {width} x {height} but the camera is calibrated for "
                f"{self._camera.width} x {self._camera.height}"
            )
        if self._reference is not None:
            east, north = gps_to_local(message.lat, message.lon, *self._reference)
            distance = math.hypot(east, north)
            if distance > self._config.max_distance_m:
                return f"GPS position {distance:.0f} m from the first photo"
        return rejection_reason(message, self._config.quality)

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
        The RANSAC-verified matching points between two photos, or None if
        they don't convincingly overlap.
        """
        config = self._config
        points_new, points_other = self._matcher.match(photo.features, other.features)
        if len(points_new) < config.min_inliers:
            return None
        affine, inliers = cv2.estimateAffinePartial2D(
            points_new,
            points_other,
            method=cv2.RANSAC,
            ransacReprojThreshold=config.ransac_threshold_px,
        )
        if affine is None:
            return None
        inliers = inliers.ravel().astype(bool)
        if inliers.sum() < config.min_inliers:
            return None

        # A photo from higher up shows more ground per pixel, so its
        # pixels map to proportionally more pixels of the other photo.
        expected_scale = photo.message.alt / other.message.alt
        scale_error = abs(math.log(scale_of(to_3x3(affine)) / expected_scale))
        if scale_error > math.log(1 + config.max_scale_error):
            return None
        return points_new[inliers], points_other[inliers]

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
        members = self._members(cluster)
        if _gps_spread(members) < self._min_trusted_spread_m():
            return True
        frame_to_local, anchored = self._georeference(cluster)
        placed = apply(frame_to_local @ pose, self._camera.principal_point)[0]
        gps = np.array([photo.east, photo.north])
        if anchored:
            error = float(np.linalg.norm(placed - gps))
        else:
            # This cluster's rotation is only a guess from yaw, which could
            # have a constant offset, so compare distances from its centre.
            centre = np.mean([[p.east, p.north] for p in members], axis=0)
            error = abs(
                float(np.linalg.norm(placed - centre) - np.linalg.norm(gps - centre))
            )
        return error <= self._config.max_gps_disagreement_m

    def _image_to_local(self, photo: _Photo) -> np.ndarray:
        frame_to_local, _ = self._georeference(photo.cluster)
        return frame_to_local @ photo.pose

    def _georeference(self, cluster: int) -> tuple[np.ndarray, bool]:
        """
        Transform from a cluster's frame to local metres, and whether it was
        fitted from GPS ("anchored") rather than guessed.

        Unanchored clusters (one photo, or all taken from about the same
        spot) get their position from GPS, their scale from the camera
        calibration, and their rotation from the photos' yaw.
        """
        anchored_fit = self._anchored_fit(cluster)
        if anchored_fit is not None:
            return anchored_fit, True

        members = self._members(cluster)
        centres = self._centres(members)
        gps = np.array([[photo.east, photo.north] for photo in members])
        metres_per_frame_px = self._metres_per_frame_px(members)
        to_origin = np.eye(3)
        to_origin[:2, 2] = -centres.mean(axis=0)
        to_gps = np.eye(3)
        to_gps[:2, 2] = gps.mean(axis=0)
        scale = np.diag([metres_per_frame_px, metres_per_frame_px, 1.0])
        rotation = np.eye(3)
        rotation[:2, :2] = self._frame_rotation(members)
        return to_gps @ rotation @ scale @ FLIP_Y @ to_origin, False

    def _anchored_fit(self, cluster: int) -> np.ndarray | None:
        """
        Best-fit rotation, scale and shift from a cluster's frame to local
        metres, matching where its photo centres are in the frame to their GPS
        positions. None if the photos are too few or too close together, by
        either GPS or the images.
        """
        if cluster in self._fit_cache:
            return self._fit_cache[cluster]

        members = self._members(cluster)
        centres = self._centres(members)
        gps = np.array([[photo.east, photo.north] for photo in members])
        fit = None
        # GPS noise alone can pass the GPS check, so the images must also show
        # the photos really are apart. Fitting noise to photos that barely
        # moved would blow up the scale.
        if (
            len(members) >= 2
            and _gps_spread(members) >= self._config.min_anchor_spread_m
            and _furthest_from_mean(centres) * self._metres_per_frame_px(members)
            >= self._min_trusted_spread_m()
        ):
            # Image y points down but north points up, so flip y first and
            # fit a reflection-free similarity.
            fit = fit_similarity(apply(FLIP_Y, centres), gps) @ FLIP_Y
        self._fit_cache[cluster] = fit
        return fit

    def _frame_rotation(self, photos: list[_Photo]) -> np.ndarray:
        """
        2 x 2 rotation from a cluster's y-flipped frame to (east, north),
        averaged over what each photo's yaw and pose say it should be.
        """
        flip = FLIP_Y[:2, :2]
        rotations = []
        for photo in photos:
            frame_to_image = np.linalg.inv(photo.pose[:2, :2]) * scale_of(photo.pose)
            rotations.append(
                image_axes_to_local(photo.message.yaw_deg) @ frame_to_image @ flip
            )
        return average_rotation(rotations)

    def _members(self, cluster: int) -> list[_Photo]:
        return [photo for photo in self._photos if photo.cluster == cluster]

    def _centres(self, photos: list[_Photo]) -> np.ndarray:
        """
        Where the ground point straight below the drone (the principal point)
        of each photo sits in its cluster frame.
        """
        point = self._camera.principal_point
        return np.array([apply(photo.pose, point)[0] for photo in photos])

    def _min_trusted_spread_m(self) -> float:
        """
        How far a cluster's photos must spread (metres) before its position
        on the map stands out from the GPS error.
        """
        return 4 * self._config.gps_error_m

    def _metres_per_px(self, photo: _Photo) -> float:
        return self._camera.metres_per_px(photo.message.alt)

    def _metres_per_frame_px(self, photos: list[_Photo]) -> float:
        """
        Ground metres per pixel of the cluster frame, from the calibration.
        """
        return float(
            np.median(
                [self._metres_per_px(photo) / scale_of(photo.pose) for photo in photos]
            )
        )


def _gps_spread(photos: list[_Photo]) -> float:
    """
    Furthest any photo's GPS position is from the group's average, in metres.
    """
    gps = np.array([[photo.east, photo.north] for photo in photos])
    return _furthest_from_mean(gps)


def _furthest_from_mean(points: np.ndarray) -> float:
    """
    Largest distance from an (N, 2) array of points to their average.
    """
    return float(np.linalg.norm(points - points.mean(axis=0), axis=1).max())
