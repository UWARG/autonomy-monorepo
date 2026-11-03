"""
End-to-end tests: synthetic missions stitched with the real XFeat matcher.
"""

import dataclasses
import json
import math
import random
import threading
from collections import deque

import cv2
import numpy as np
import pytest

from src.image_stitcher.camera import Camera
from src.image_stitcher.features import XFeatMatcher
from src.image_stitcher.geo import FLIP_Y, apply, fit_similarity, gps_to_local
from src.image_stitcher.stitcher import ImageStitcher, StitcherConfig
from src.image_stitcher.synthetic import lawn_mower_mission, make_ground

GPS_NOISE_M = 1.0


@pytest.fixture(scope="module")
def matcher():
    return XFeatMatcher()


@pytest.fixture(scope="module")
def camera():
    return Camera.load()


@pytest.fixture(scope="module")
def ground():
    return make_ground(width_px=1600, height_px=1200)


@pytest.fixture(scope="module")
def mission(ground, camera):
    """
    Three drones, 1 m GPS error, small heading and altitude wobble, and the
    heading flipping 180 degrees every row. Photos have the real camera's
    lens distortion. The stitcher is told none of the headings.
    """
    photos = lawn_mower_mission(
        ground,
        camera,
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
def stitched(mission, matcher, camera):
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
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
    # Placements hold undistorted copies of the messages.
    placements = {message.image_id: placed for message, placed in stitcher.placements()}
    assert len(placements) == stitcher.image_count, "image_ids must be unique"
    corners_px = np.array([[0, 0], [639, 0], [639, 479], [0, 479]])

    stitched_points, true_points = [], []
    for photo in photos:
        placed = placements[photo.message.image_id]
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


def test_ground_resolution_comes_from_calibration(stitched, camera):
    # Altitudes wobble by 0.3 m around 25 m, so the median is close to 25 m.
    true_gsd = 25.0 / camera.focal_px
    assert stitched.render().resolution_m == pytest.approx(true_gsd, rel=0.01)


def test_photo_of_wrong_size_is_skipped(mission, matcher, camera):
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    message = mission[0].message
    resized = dataclasses.replace(message, img=message.img[:240])
    assert not stitcher.add(resized)
    assert stitcher.image_count == 0


def test_run_loop_consumes_deque_and_saves(mission, matcher, camera, tmp_path):
    queue: deque = deque()
    stop = threading.Event()
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    worker = threading.Thread(
        target=stitcher.run,
        args=(queue, stop),
        kwargs={"output_dir": tmp_path, "save_every": 2, "poll_interval_s": 0.01},
    )
    worker.start()

    bad = dataclasses.replace(mission[0].message, lat=math.nan, lon=math.nan)
    queue.append(bad)
    for photo in mission[:5]:
        queue.append(photo.message)
    stop.set()
    worker.join(timeout=120)

    assert not worker.is_alive()
    assert not queue
    assert stitcher.image_count == 5
    # Saved after photos 2 and 4, and once more with all 5 when stopping.
    maps = sorted(path.name for path in tmp_path.glob("map_*.png"))
    assert maps == ["map_001.png", "map_002.png", "map_003.png"]
    counts = [
        json.loads((tmp_path / name).with_suffix(".json").read_text())["image_count"]
        for name in maps
    ]
    assert counts == [2, 4, 5]


def test_run_loop_skips_duplicate_final_save(mission, matcher, camera, tmp_path):
    # The last periodic save already holds all 4 photos, so stopping must not
    # write an identical map_003.
    queue = deque(photo.message for photo in mission[:4])
    stop = threading.Event()
    stop.set()
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    stitcher.run(queue, stop, output_dir=tmp_path, save_every=2)

    maps = sorted(path.name for path in tmp_path.glob("map_*.png"))
    assert maps == ["map_001.png", "map_002.png"]


def test_process_queue_drains_everything(mission, matcher, camera):
    queue = deque(photo.message for photo in mission[:3])
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    assert stitcher.process_queue(queue) == 3
    assert not queue
    assert stitcher.process_queue(queue) == 0


def test_unusable_photos_in_queue_are_rejected(mission, matcher, camera):
    def bad(photo, kind, **changes):
        message = photo.message
        return dataclasses.replace(
            message, image_id=f"{kind}_{message.image_id}", **changes
        )

    def scaled(img, gain):
        return np.clip(img.astype(float) * gain, 0, 255).astype(np.uint8)

    motion = np.zeros((15, 15), np.float32)
    motion[7, :] = 1 / 15
    good = [photo.message for photo in mission[:10]]
    spare = mission[10:]
    unusable = [
        bad(spare[0], "blurry", img=cv2.GaussianBlur(spare[0].message.img, (0, 0), 2)),
        bad(spare[1], "shaky", img=cv2.filter2D(spare[1].message.img, -1, motion)),
        bad(spare[2], "bright", img=scaled(spare[2].message.img, 3.0)),
        bad(spare[3], "dark", img=scaled(spare[3].message.img, 0.1)),
        bad(spare[4], "rolled", roll_deg=12.0),
        bad(spare[5], "pitched", pitch_deg=-8.0),
        bad(spare[6], "noattitude", roll_deg=math.nan, pitch_deg=math.nan),
    ]
    messages = good + unusable
    random.Random(3).shuffle(messages)

    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    assert stitcher.process_queue(deque(messages)) == len(messages)

    assert stitcher.image_count == len(good)
    placed = {message.image_id for message, _ in stitcher.placements()}
    assert placed == {message.image_id for message in good}
    assert stitcher.render().image_count == len(good)

    reasons = {image_id: reason for _, image_id, reason in stitcher.rejections}
    assert set(reasons) == {message.image_id for message in unusable}
    expected = {
        "blurry": "blurry",
        "shaky": "blurry",
        "bright": "overexposed",
        "dark": "underexposed",
        "rolled": "tilted",
        "pitched": "tilted",
        "noattitude": "no roll/pitch",
    }
    for image_id, reason in reasons.items():
        assert reason.startswith(expected[image_id.split("_")[0]]), image_id


def test_separate_areas_stay_separate_then_merge(ground, matcher, camera):
    # Two drones start at opposite ends of the field; their photos can't be
    # matched until the gap between them is photographed.
    photos = lawn_mower_mission(
        ground, camera, altitude_m=25.0, gps_noise_m=0.5, seed=3
    )
    first_row = [p for p in photos if p.true_north == photos[0].true_north]
    west_end = first_row[:2]
    east_end = first_row[-2:]
    middle = first_row[2:-2]

    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    for photo in west_end + east_end:
        stitcher.add(photo.message)
    assert stitcher.cluster_count == 2
    stitcher.render()

    for photo in middle:
        stitcher.add(photo.message)
    assert stitcher.cluster_count == 1
    _, relative = placement_errors(stitcher, first_row, ground)
    assert relative < 0.1


def test_photo_with_no_overlap_is_still_placed_by_gps(ground, matcher, camera):
    photos = lawn_mower_mission(ground, camera, altitude_m=25.0, seed=4)
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    stitcher.add(photos[0].message)
    stitched_map = stitcher.render()
    assert stitched_map.image_count == 1
    assert stitched_map.coverage.any()


def test_flight_at_two_altitudes_merges(ground, matcher, camera):
    low = lawn_mower_mission(ground, camera, altitude_m=22.0, overlap=0.5, seed=5)
    high = [
        dataclasses.replace(
            photo,
            message=dataclasses.replace(
                photo.message, image_id=f"high_{photo.message.image_id}"
            ),
        )
        for photo in lawn_mower_mission(
            ground, camera, altitude_m=28.0, overlap=0.5, seed=6
        )
    ]
    photos = low[:4] + high[:4]
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    for photo in photos:
        stitcher.add(photo.message)
    assert stitcher.cluster_count == 1
    _, relative = placement_errors(stitcher, photos, ground)
    assert relative < 0.1


def test_flip_y_is_its_own_inverse():
    np.testing.assert_allclose(FLIP_Y @ FLIP_Y, np.eye(3))


def test_repeating_ground_does_not_fool_it(ground, matcher, camera):
    # The east half of the field is an exact copy of the west half, so a
    # photo of the east half looks just like a spot 40 m further west. Its
    # GPS has to overrule that wrong image match.
    texture = ground.texture.copy()
    texture[:, 800:] = texture[:, :800]
    repeating = type(ground)(texture, ground.resolution_m, ground.origin)
    photos = lawn_mower_mission(
        repeating, camera, altitude_m=25.0, gps_noise_m=1.0, seed=7
    )
    west = [photo for photo in photos if photo.true_east < 30]
    east = next(photo for photo in photos if 55 < photo.true_east < 65)

    # Told the GPS is good to about 2 m, so a cluster spanning 8 m+ is trusted.
    stitcher = ImageStitcher(
        StitcherConfig(gps_error_m=2.0), matcher=matcher, camera=camera
    )
    for photo in west:
        stitcher.add(photo.message)
    stitcher.add(east.message)

    assert stitcher.cluster_count == 2
    # Placed by GPS alone. Its heading is unknown, so only check its centre.
    message, placed = stitcher.placements()[-1]
    assert message.image_id == east.message.image_id
    offset = np.array(gps_to_local(*stitcher.reference, *repeating.origin))
    centre = apply(placed, [camera.principal_point])[0] + offset
    assert np.linalg.norm(centre - [east.true_east, east.true_north]) < 5.0
