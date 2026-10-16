"""
End-to-end tests: synthetic missions stitched with the real XFeat matcher.
"""

import math
import random
import threading
from collections import deque

import numpy as np
import pytest

from src.image_stitcher.features import XFeatMatcher
from src.image_stitcher.geo import FLIP_Y, apply, fit_similarity, gps_to_local
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.stitcher import ImageStitcher, StitcherConfig
from src.image_stitcher.synthetic import lawn_mower_mission, make_ground

GPS_NOISE_M = 1.0


@pytest.fixture(scope="module")
def matcher():
    return XFeatMatcher()


@pytest.fixture(scope="module")
def ground():
    return make_ground(width_px=1600, height_px=1200)


@pytest.fixture(scope="module")
def mission(ground):
    """
    Three drones, 1 m GPS error, small heading and altitude wobble, and the
    heading flipping 180 degrees every row. The stitcher is told none of
    the headings or the camera FOV.
    """
    photos = lawn_mower_mission(
        ground,
        altitude_m=25.0,
        gps_noise_m=GPS_NOISE_M,
        altitude_noise_m=0.3,
        heading_noise_deg=3.0,
        drone_count=3,
        seed=1,
    )
    random.Random(2).shuffle(photos)
    return photos


@pytest.fixture(scope="module")
def stitched(mission, matcher):
    stitcher = ImageStitcher(matcher=matcher)
    for photo in mission:
        assert stitcher.add(photo.message)
    return stitcher


def placement_errors(stitcher, photos, ground):
    """
    Where the stitcher put each photo's corners vs where they truly are.

    Returns (absolute, relative) mean corner errors in metres. Absolute is
    against true ground positions. Relative first removes the single best
    rotation/scale/shift of the whole map, leaving only how well photos line
    up with each other: what makes the map sharp or ghosted.
    """
    # Stitcher metres are relative to its reference; truth is relative to
    # the ground's north west corner.
    offset = np.array(gps_to_local(*stitcher.reference, *ground.origin))
    placements = {id(message): placed for message, placed in stitcher.placements()}
    corners_px = np.array([[0, 0], [639, 0], [639, 479], [0, 479]])

    stitched_points, true_points = [], []
    for photo in photos:
        placed = placements[id(photo.message)]
        stitched_points.append(apply(placed, corners_px) + offset)
        true_points.append(apply(photo.true_image_to_local, corners_px))
    stitched_points = np.concatenate(stitched_points)
    true_points = np.concatenate(true_points)

    absolute = np.linalg.norm(stitched_points - true_points, axis=1).mean()
    best_fit = fit_similarity(stitched_points, true_points)
    relative = np.linalg.norm(apply(best_fit, stitched_points) - true_points, axis=1)
    return absolute, relative.mean()


def test_mission_forms_one_cluster(stitched, mission):
    assert stitched.image_count == len(mission)
    assert stitched.cluster_count == 1


def test_photos_line_up_far_better_than_gps(stitched, mission, ground):
    absolute, relative = placement_errors(stitched, mission, ground)
    # GPS alone would misalign photos by about GPS_NOISE_M. Image matching
    # lines them up to within a few centimetres (about one pixel).
    assert relative < 0.1 * GPS_NOISE_M
    # GPS errors average out over the whole map instead of adding up.
    assert absolute < GPS_NOISE_M


def test_rendered_map_matches_ground(stitched, ground):
    stitched_map = stitched.render()
    assert stitched_map.coverage.mean() > 0.9
    # Map spans the mission area (about 80 x 60 m).
    width_m = stitched_map.rgb.shape[1] * stitched_map.resolution_m
    height_m = stitched_map.rgb.shape[0] * stitched_map.resolution_m
    assert width_m == pytest.approx(ground.width_m, rel=0.1)
    assert height_m == pytest.approx(ground.height_m, rel=0.1)


def test_learns_ground_resolution_without_fov(stitched, mission):
    # The true FOV is 70 degrees: 25 m altitude -> 0.0547 m per pixel.
    true_gsd = 2 * 25.0 * math.tan(math.radians(35)) / 640
    assert stitched.render().resolution_m == pytest.approx(true_gsd, rel=0.03)


def test_run_loop_consumes_deque_and_saves(mission, matcher, tmp_path):
    queue: deque = deque()
    stop = threading.Event()
    stitcher = ImageStitcher(matcher=matcher)
    output = tmp_path / "map.png"
    worker = threading.Thread(
        target=stitcher.run,
        args=(queue, stop),
        kwargs={"output_path": output, "poll_interval_s": 0.01},
    )
    worker.start()

    bad = ImageMessage(math.nan, math.nan, math.nan, mission[0].message.img)
    queue.append(bad)
    for photo in mission[:5]:
        queue.append(photo.message)
    stop.set()
    worker.join(timeout=120)

    assert not worker.is_alive()
    assert not queue
    assert stitcher.image_count == 5
    assert output.exists()
    assert output.with_suffix(".json").exists()


def test_process_queue_drains_everything(mission, matcher):
    queue = deque(photo.message for photo in mission[:3])
    stitcher = ImageStitcher(matcher=matcher)
    assert stitcher.process_queue(queue) == 3
    assert not queue
    assert stitcher.process_queue(queue) == 0


def test_separate_areas_stay_separate_then_merge(ground, matcher):
    # Two drones start at opposite ends of the field; their photos can't be
    # matched until the gap between them is photographed.
    photos = lawn_mower_mission(ground, altitude_m=25.0, gps_noise_m=0.5, seed=3)
    first_row = [p for p in photos if p.true_north == photos[0].true_north]
    west_end = first_row[:2]
    east_end = first_row[-2:]
    middle = first_row[2:-2]

    stitcher = ImageStitcher(matcher=matcher)
    for photo in west_end + east_end:
        stitcher.add(photo.message)
    assert stitcher.cluster_count == 2
    stitcher.render()

    for photo in middle:
        stitcher.add(photo.message)
    assert stitcher.cluster_count == 1
    _, relative = placement_errors(stitcher, first_row, ground)
    assert relative < 0.1


def test_photo_with_no_overlap_is_still_placed_by_gps(ground, matcher):
    photos = lawn_mower_mission(ground, altitude_m=25.0, seed=4)
    stitcher = ImageStitcher(matcher=matcher)
    stitcher.add(photos[0].message)
    stitched_map = stitcher.render()
    assert stitched_map.image_count == 1
    assert stitched_map.coverage.any()


def test_flight_at_two_altitudes_merges(ground, matcher):
    low = lawn_mower_mission(ground, altitude_m=22.0, overlap=0.5, seed=5)
    high = lawn_mower_mission(ground, altitude_m=28.0, overlap=0.5, seed=6)
    photos = low[:4] + high[:4]
    stitcher = ImageStitcher(matcher=matcher)
    for photo in photos:
        stitcher.add(photo.message)
    assert stitcher.cluster_count == 1
    _, relative = placement_errors(stitcher, photos, ground)
    assert relative < 0.1


def test_flip_y_is_its_own_inverse():
    np.testing.assert_allclose(FLIP_Y @ FLIP_Y, np.eye(3))


def test_repeating_ground_does_not_fool_it(ground, matcher):
    # The east half of the field is an exact copy of the west half, so a
    # photo of the east half looks just like a spot 40 m further west. Its
    # GPS has to overrule that wrong image match.
    texture = ground.texture.copy()
    texture[:, 800:] = texture[:, :800]
    repeating = type(ground)(texture, ground.resolution_m, ground.origin)
    photos = lawn_mower_mission(repeating, altitude_m=25.0, gps_noise_m=1.0, seed=7)
    west = [photo for photo in photos if photo.true_east < 30]
    east = next(photo for photo in photos if 55 < photo.true_east < 65)

    # Told the GPS is good to about 2 m, so a cluster spanning 8 m+ is trusted.
    stitcher = ImageStitcher(StitcherConfig(gps_error_m=2.0), matcher=matcher)
    for photo in west:
        stitcher.add(photo.message)
    stitcher.add(east.message)

    assert stitcher.cluster_count == 2
    # Placed by GPS alone. Its heading is unknown, so only check its centre.
    message, placed = stitcher.placements()[-1]
    assert message is east.message
    offset = np.array(gps_to_local(*stitcher.reference, *repeating.origin))
    centre = apply(placed, [[319.5, 239.5]])[0] + offset
    assert np.linalg.norm(centre - [east.true_east, east.true_north]) < 5.0
