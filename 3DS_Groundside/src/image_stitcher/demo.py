"""
Stitches a synthetic multi-drone lawn mower mission and saves the result.

Usage (from 3DS_Groundside/):
    uv run python -m src.image_stitcher.demo [output_dir] [--gps-noise M] [--drones N]
"""

from __future__ import annotations

import argparse
import logging
import random
import threading
import time
from collections import deque
from pathlib import Path

import cv2

from src.image_stitcher.stitcher import ImageStitcher
from src.image_stitcher.synthetic import lawn_mower_mission, make_ground


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", nargs="?", default="stitcher_demo")
    parser.add_argument("--gps-noise", type=float, default=1.0, help="metres")
    parser.add_argument("--drones", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    ground = make_ground(seed=args.seed)
    photos = lawn_mower_mission(
        ground,
        altitude_m=25.0,
        gps_noise_m=args.gps_noise,
        altitude_noise_m=0.3,
        heading_noise_deg=3.0,
        drone_count=args.drones,
        seed=args.seed,
    )
    # Drones finish photos at different times, so they arrive interleaved.
    random.Random(args.seed).shuffle(photos)
    cv2.imwrite(
        str(output_dir / "ground_truth.png"),
        cv2.cvtColor(ground.texture, cv2.COLOR_RGB2BGR),
    )

    queue: deque = deque()
    stop = threading.Event()
    stitcher = ImageStitcher()
    worker = threading.Thread(
        target=stitcher.run,
        args=(queue, stop),
        kwargs={"output_path": output_dir / "map.png", "save_every": 20},
    )
    worker.start()

    started = time.perf_counter()
    for photo in photos:
        queue.append(photo.message)
    stop.set()
    worker.join()

    print(
        f"Stitched {stitcher.image_count} photos into "
        f"{stitcher.cluster_count} cluster(s) in "
        f"{time.perf_counter() - started:.1f} s -> {output_dir / 'map.png'}"
    )


if __name__ == "__main__":
    main()
