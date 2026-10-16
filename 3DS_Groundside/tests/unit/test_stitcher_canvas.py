import json
import math

import cv2
import numpy as np
import pytest

from src.image_stitcher.canvas import Canvas
from src.image_stitcher.files import load_photo_run, write_map
from src.image_stitcher.geo import gps_to_local
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.stitcher import StitchedMap
from src.image_stitcher.synthetic import make_ground, take_photo

# From 20 m this sees 0.05 m per pixel across 640 px, matching the ground.
FOV_DEG = math.degrees(2 * math.atan(0.8))


@pytest.fixture(scope="module")
def ground():
    return make_ground(width_px=1600, height_px=1200)


def test_photo_painted_at_true_position_reproduces_ground(ground):
    rgb, image_to_local = take_photo(ground, FOV_DEG, 40.0, -30.0, 20.0, 35.0)
    # Local metres here are relative to the ground's north west corner.
    canvas = Canvas(ground.resolution_m, ground.origin)
    canvas.add(rgb, image_to_local)
    mosaic, coverage = canvas.render()

    # Canvas pixel (0, 0) is ground pixel (x0, y0).
    x0, y0 = (round(v) for v in _ground_pixel(canvas, 0, 0, ground))
    truth = ground.texture[y0 : y0 + canvas.height, x0 : x0 + canvas.width]
    error = np.abs(mosaic.astype(int) - truth.astype(int))[coverage].mean()
    # Rotating twice (taking the photo, then painting it) blurs pixels a
    # little; a photo one pixel off would score far higher.
    assert error < 6.0
    assert coverage.sum() == pytest.approx(640 * 480, rel=0.02)


def _ground_pixel(canvas, column, row, ground):
    """
    Ground texture pixel shown at a canvas pixel, via local metres.
    """
    res = canvas.resolution_m
    east = (canvas._left_px + column + 0.5) * res
    north = (canvas._top_px - row - 0.5) * res
    return east / ground.resolution_m - 0.5, -north / ground.resolution_m - 0.5


def test_canvas_grows_and_bounds_follow():
    canvas = Canvas(1.0, (43.0, -80.0))
    image = np.full((10, 10, 3), 200, dtype=np.uint8)
    square = np.diag([1.0, -1.0, 1.0])
    canvas.add(image, square)
    shifted = square.copy()
    shifted[:2, 2] = [50.0, -50.0]
    canvas.add(image, shifted)
    assert canvas.width >= 59
    assert canvas.height >= 59

    bounds = canvas.bounds()
    west, north = gps_to_local(*bounds.north_west, 43.0, -80.0)
    east, south = gps_to_local(*bounds.south_east, 43.0, -80.0)
    assert west <= 0 and north >= 0
    assert east >= 59 - 1e-6 and south <= -59 + 1e-6


def test_canvas_refuses_runaway_size():
    canvas = Canvas(0.05, (43.0, -80.0), max_size_px=1000)
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    huge = np.diag([100.0, -100.0, 1.0])
    with pytest.raises(ValueError):
        canvas.add(image, huge)


def test_message_validity():
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    assert ImageMessage(43.0, -80.0, 20.0, image).has_valid_position()
    assert not ImageMessage(math.nan, math.nan, math.nan, image).has_valid_position()
    assert not ImageMessage(43.0, -80.0, 0.0, image).has_valid_position()
    assert ImageMessage(43.0, -80.0, 20.0, image).has_valid_image()
    assert not ImageMessage(43.0, -80.0, 20.0, image[..., 0]).has_valid_image()


def test_write_map(tmp_path):
    canvas = Canvas(1.0, (43.0, -80.0))
    canvas.add(np.full((10, 20, 3), 90, dtype=np.uint8), np.diag([1.0, -1.0, 1.0]))
    rgb, coverage = canvas.render()
    stitched = StitchedMap(rgb, coverage, canvas.bounds(), 1.0, 1)

    write_map(tmp_path / "out" / "map.png", stitched)

    saved = cv2.imread(str(tmp_path / "out" / "map.png"), cv2.IMREAD_UNCHANGED)
    assert saved.shape == (*rgb.shape[:2], 4)
    assert (saved[..., 3] == coverage * 255).all()
    metadata = json.loads((tmp_path / "out" / "map.json").read_text())
    assert metadata["image_count"] == 1
    north_west = metadata["corners_lat_lon"]["north_west"]
    south_east = metadata["corners_lat_lon"]["south_east"]
    assert north_west[0] > south_east[0]
    assert north_west[1] < south_east[1]


def test_load_photo_run_reads_3ds_camera_format(tmp_path):
    images = [np.full((8, 8, 3), value, dtype=np.uint8) for value in (10, 20)]
    with (tmp_path / "photos.jsonl").open("w") as index:
        for number, image in enumerate(images, start=1):
            name = f"photo_{number:04d}.png"
            cv2.imwrite(str(tmp_path / name), image)
            position = {"lat": 43.0 + number, "lon": -80.0, "alt": 20.0}
            record = {
                "stamp": "",
                "file": name,
                "setpoint": position,
                "position": position,
            }
            index.write(json.dumps(record) + "\n")

    loaded = list(load_photo_run(tmp_path, drone_id="drone-01"))

    assert [message.image_id for message in loaded] == [
        "photo_0001.png",
        "photo_0002.png",
    ]
    assert loaded[1].lat == 45.0
    assert loaded[0].drone_id == "drone-01"
    np.testing.assert_array_equal(loaded[1].img, images[1])
