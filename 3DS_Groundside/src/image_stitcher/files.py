"""
Reading drone photo runs and writing finished maps.
"""

from __future__ import annotations

import json
import logging
from collections.abc import Iterator
from pathlib import Path
from typing import TYPE_CHECKING

import cv2

from src.image_stitcher.message import ImageMessage

if TYPE_CHECKING:
    from src.image_stitcher.stitcher import StitchedMap

logger = logging.getLogger(__name__)


def write_map(path: Path, stitched: StitchedMap) -> None:
    """
    Writes the map as a PNG (transparent where no photo covers the ground)
    and <name>.json holding its GPS corners and resolution.
    """
    bgra = cv2.cvtColor(stitched.rgb, cv2.COLOR_RGB2BGRA)
    bgra[..., 3] = stitched.coverage * 255
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), bgra):
        raise OSError(f"failed to write map to {path}")

    bounds = stitched.bounds
    metadata = {
        "resolution_m_per_px": stitched.resolution_m,
        "image_count": stitched.image_count,
        "width_px": stitched.rgb.shape[1],
        "height_px": stitched.rgb.shape[0],
        "corners_lat_lon": {
            "north_west": bounds.north_west,
            "north_east": bounds.north_east,
            "south_east": bounds.south_east,
            "south_west": bounds.south_west,
        },
    }
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2))


def load_photo_run(run_dir: Path, drone_id: str = "") -> Iterator[ImageMessage]:
    """
    Reads a photo run saved by the 3ds airside camera node: photo_NNNN.png
    files indexed by photos.jsonl, using each record's measured `position`.
    """
    with (run_dir / "photos.jsonl").open() as index:
        for line in index:
            if not line.strip():
                continue
            record = json.loads(line)
            bgr = cv2.imread(str(run_dir / record["file"]), cv2.IMREAD_COLOR)
            if bgr is None:
                logger.warning("Could not read %s", record["file"])
                continue
            position = record["position"]
            yield ImageMessage(
                lat=position["lat"],
                lon=position["lon"],
                alt=position["alt"],
                img=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                drone_id=drone_id,
                image_id=record["file"],
            )
