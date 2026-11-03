"""
Measures the stitcher on synthetic missions with known ground truth. Produces
the "Accuracy" figures in the README (simulated).

Usage (from 3DS_Groundside/, about 7 minutes for the default 5 seeds):
    uv run python -m src.image_stitcher.evaluate [--seeds N]
"""

from __future__ import annotations

import argparse
import logging
import random
import time

import numpy as np

from src.image_stitcher.camera import Camera
from src.image_stitcher.features import XFeatMatcher
from src.image_stitcher.geo import apply, fit_similarity, gps_to_local
from src.image_stitcher.stitcher import ImageStitcher
from src.image_stitcher.synthetic import (
    SyntheticGround,
    SyntheticPhoto,
    lawn_mower_mission,
    make_ground,
)

GPS_ERRORS_M = (1.0, 3.0)


def placement_errors(
    stitcher: ImageStitcher, photos: list[SyntheticPhoto], ground: SyntheticGround
) -> tuple[float, float]:
    """
    Mean distance (metres) between where the stitcher put each placed photo's
    corners and where they truly are: (absolute, relative). Relative first
    removes the single best rotation/scale/shift of the whole map, leaving
    only how well photos line up with each other.
    """
    # Stitcher metres are relative to its reference; truth is relative to
    # the ground's north west corner.
    offset = np.array(gps_to_local(*stitcher.reference, *ground.origin))
    placed = {message.image_id: pose for message, pose in stitcher.placements()}
    stitched, truth = [], []
    for photo in photos:
        if photo.message.image_id not in placed:
            continue  # rejected; counted separately
        height, width = photo.message.img.shape[:2]
        corners_px = np.array(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]]
        )
        stitched.append(apply(placed[photo.message.image_id], corners_px) + offset)
        truth.append(apply(photo.true_image_to_local, corners_px))
    stitched, truth = np.concatenate(stitched), np.concatenate(truth)
    absolute = np.linalg.norm(stitched - truth, axis=1).mean()
    best_fit = fit_similarity(stitched, truth)
    relative = np.linalg.norm(apply(best_fit, stitched) - truth, axis=1).mean()
    return float(absolute), float(relative)


def run_mission(
    matcher: XFeatMatcher, camera: Camera, gps_error_m: float, seed: int
) -> dict[str, float]:
    """
    Stitches one 3-drone mission, photos arriving shuffled, and scores it.
    """
    ground = make_ground(seed=seed)
    photos = lawn_mower_mission(
        ground,
        camera,
        altitude_m=25.0,
        gps_noise_m=gps_error_m,
        altitude_noise_m=0.3,
        heading_noise_deg=3.0,
        drone_count=3,
        seed=seed,
    )
    random.Random(seed).shuffle(photos)
    stitcher = ImageStitcher(matcher=matcher, camera=camera)
    started = time.perf_counter()
    for photo in photos:
        stitcher.add(photo.message)
    seconds_per_photo = (time.perf_counter() - started) / len(photos)
    # Score the poses the map would be drawn with (render() refines first).
    stitcher.refine()
    absolute, relative = placement_errors(stitcher, photos, ground)
    return {
        "photos": len(photos),
        "rejected": len(stitcher.rejections),
        "clusters": stitcher.cluster_count,
        "absolute_m": absolute,
        "relative_m": relative,
        "s_per_photo": seconds_per_photo,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=5)
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(message)s")

    camera = Camera.load()
    matcher = XFeatMatcher()
    print("gps_m seed photos rejected clusters abs_m rel_m s_per_photo")
    results: dict[float, list[dict[str, float]]] = {}
    for gps_error in GPS_ERRORS_M:
        for seed in range(args.seeds):
            result = run_mission(matcher, camera, gps_error, seed)
            results.setdefault(gps_error, []).append(result)
            print(
                f"{gps_error:.0f} {seed} {result['photos']} {result['rejected']} "
                f"{result['clusters']} {result['absolute_m']:.3f} "
                f"{result['relative_m']:.3f} {result['s_per_photo']:.2f}",
                flush=True,
            )

    print()
    print("Simulated, min-max over seeds:")
    for gps_error, rows in results.items():
        one_map = sum(row["clusters"] == 1 for row in rows)
        rejected = sum(row["rejected"] for row in rows)
        print(
            f"GPS {gps_error:.0f} m: photos line up {_span(rows, 'relative_m')} m, "
            f"map position {_span(rows, 'absolute_m')} m, "
            f"one map {one_map}/{len(rows)}, {rejected} rejected, "
            f"{_span(rows, 's_per_photo')} s per photo"
        )


def _span(rows: list[dict[str, float]], key: str) -> str:
    values = [row[key] for row in rows]
    return f"{min(values):.3f}-{max(values):.3f}"


if __name__ == "__main__":
    main()
