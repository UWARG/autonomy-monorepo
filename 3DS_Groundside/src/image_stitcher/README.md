# Image Stitcher

Groundside orthomosaic stitcher for 3DS ([#179](https://github.com/UWARG/autonomy-monorepo/issues/179)).
It takes photos from a fleet of drones, arriving in any order, and builds one
top-down map of the whole area.

Each photo only needs its GPS position (lat, lon, alt). Drone heading and
camera field of view are not needed: they are worked out from the photos.

For a full beginner-friendly explanation of how it works, see
[OVERVIEW.md](OVERVIEW.md).

## Usage

```python
from collections import deque
from pathlib import Path

from src.image_stitcher.message import ImageMessage
from src.image_stitcher.stitcher import ImageStitcher

queue = deque()
stitcher = ImageStitcher()

# Whatever receives photos from the drones appends to the queue...
queue.append(ImageMessage(lat=43.4723, lon=-80.5449, alt=25.0, img=rgb_array))

# ...and the stitcher drains it.
stitcher.process_queue(queue)
stitcher.save(Path("map.png"))  # map.png + map.json (GPS corners, m/px)
```

`ImageStitcher.run(queue, stop_event, output_path=...)` does the same in a loop
on a worker thread, saving periodically, until `stop_event` is set.
`files.load_photo_run(run_dir)` reads a photo run saved by the 3ds airside
camera node (`photos.jsonl` + PNGs).

The first run needs internet: XFeat is downloaded from its GitHub repo with
`torch.hub` (pinned commit, about 6 MB) and cached in `~/.cache/torch/hub`.
Run the demo once on the groundside laptop before going to the field.

Demo on a synthetic 3-drone mission (writes `stitcher_demo/map.png` and
`ground_truth.png` for comparison):

```bash
cd 3DS_Groundside
uv run python -m src.image_stitcher.demo stitcher_demo --gps-noise 2
```

## How it works

For each new photo:

1. **Find neighbours.** GPS picks the already-placed photos close enough to
   overlap it.
2. **Match features.** [XFeat](https://github.com/verlab/accelerated_features)
   finds the same ground points in the new photo and each neighbour. XFeat
   is not rotation invariant and lawn mower paths flip 180 degrees every row,
   so features are extracted at all four quarter turns and the best is kept.
3. **Verify.** RANSAC fits the shift, rotation and scale between the photos
   and throws out wrong matches. A match is rejected if it has too few
   supporting points, if its scale disagrees with the altitude ratio, or if it
   would put the photo far from its GPS.
4. **Place.** The photo joins the group ("cluster") of photos it lines up
   with, merging clusters if it connects two. A photo that matches nothing
   starts its own cluster at its GPS position.
5. **Refine.** Every 10 photos and before drawing, all poses in a cluster are
   re-solved together (`refine.py`) so small errors don't pile up.

When drawing, each cluster gets one best-fit rotation/scale/shift against all
of its photos' GPS positions, so GPS errors average out across the map
instead of making photos disagree with each other. Overlaps are blended with
weights that fade towards each photo's edges, hiding seams.

| File | Purpose |
|---|---|
| `message.py` | `ImageMessage`: what arrives on the queue |
| `stitcher.py` | `ImageStitcher`: matching, clustering, placing, rendering |
| `features.py` | XFeat wrapper, rotated feature extraction |
| `refine.py` | Joint least-squares refinement of photo poses |
| `canvas.py` | The growing map image and blending |
| `geo.py` | GPS / local metre / pixel geometry |
| `files.py` | Writing maps, reading 3ds airside photo runs |
| `synthetic.py` | Fake missions with known ground truth, for tests and the demo |

## Accuracy

Measured on synthetic 120 x 90 m missions (50 photos, 3 drones, shuffled
arrival, unknown heading, ±0.5 m altitude error), 5 random fields each:

| GPS error | Photos line up with each other | Map position vs truth | Connected map |
|---|---|---|---|
| 1 m | 3 to 6 cm (about 1 px) | 0.15 to 0.45 m | 5 / 5 |
| 3 m | 3 to 6 cm (about 1 px) | 0.5 to 1.3 m | 5 / 5 |

About 0.5 s per photo on a laptop CPU.

## Configuration

`StitcherConfig` holds the tuning knobs, each documented in `stitcher.py`.
The ones most likely to need changing for real flights:

- `gps_error_m`: set to the real GPS accuracy.
- `min_inliers`: raise it if wrong matches appear over repetitive ground.
- `resolution_m`: fixed output metres per pixel (default: the photos' own).

## Limitations / future work

- Assumes flat ground and a straight-down camera. Hills, buildings and tilted
  photos will show small misalignments.
- No lens distortion correction yet. Camera calibration
  ([#212](https://github.com/UWARG/autonomy-monorepo/issues/212)) would
  allow undistorting photos first.
- A photo that overlaps nothing is placed by GPS alone, facing north,
  because its heading is unknown. It snaps into place once neighbours arrive.
- The map's absolute position is only as good as the averaged GPS (see
  Accuracy). Ground control points would be needed for better.
- Rendering redraws every photo, and refinement cost grows with cluster size.
  Fine for hundreds of photos; thousands would need tiling.
- Tested on synthetic images only; needs a run on real drone photos.

## Tests

```bash
uv run --extra dev pytest tests/unit             # fast, no XFeat
uv run --extra dev pytest tests/integration      # XFeat end to end, ~30 s
```
