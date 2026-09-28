import json
from datetime import datetime

import numpy as np
from airside_interfaces.msg import Coordinate as CoordinateMsg
from PIL import Image as PILImage

from camera_driver.camera_node import PhotoLog


def test_photo_log_creates_run_directory(tmp_path):
    log = PhotoLog(tmp_path, datetime(2026, 9, 24, 12, 0, 0))
    assert log.run_dir == tmp_path / "run_2026-09-24T12-00-00"
    assert log.run_dir.is_dir()


def test_photo_log_save_writes_image_and_index(tmp_path):
    log = PhotoLog(tmp_path, datetime(2026, 9, 24, 12, 0, 0))
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    rgb[0, 0] = (255, 0, 0)

    photo_file = log.save(
        rgb,
        setpoint=CoordinateMsg(lat=1.0, lon=2.0, alt=10.0),
        position=CoordinateMsg(lat=1.1, lon=2.1, alt=9.5),
        stamp="2026-09-24T12:00:05",
    )

    assert photo_file == log.run_dir / "photo_0001.png"
    saved = np.asarray(PILImage.open(photo_file))
    assert saved.shape == (4, 6, 3)
    assert tuple(saved[0, 0]) == (255, 0, 0)

    record = json.loads(log.index_file.read_text().splitlines()[0])
    assert record == {
        "stamp": "2026-09-24T12:00:05",
        "file": "photo_0001.png",
        "setpoint": {"lat": 1.0, "lon": 2.0, "alt": 10.0},
        "position": {"lat": 1.1, "lon": 2.1, "alt": 9.5},
    }


def test_photo_log_numbers_photos_sequentially(tmp_path):
    log = PhotoLog(tmp_path, datetime(2026, 9, 24, 12, 0, 0))
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    coordinate = CoordinateMsg(lat=0.0, lon=0.0, alt=0.0)

    first = log.save(rgb, setpoint=coordinate, position=coordinate, stamp="a")
    second = log.save(rgb, setpoint=coordinate, position=coordinate, stamp="b")

    assert first.name == "photo_0001.png"
    assert second.name == "photo_0002.png"
    assert len(log.index_file.read_text().splitlines()) == 2
