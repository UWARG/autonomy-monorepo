# Image Stitcher

Groundside orthomosaic stitcher for 3DS ([#179](https://github.com/UWARG/autonomy-monorepo/issues/179)).
It takes photos from a fleet of drones, arriving in any order, and builds one
top-down map of the whole area.

Each photo needs its GPS position (lat, lon, alt) and the drone's roll, pitch
and yaw (degrees clockwise from north that the top of the image faces, either
-180 to 180 or 0 to 360).

## Usage

```python
from collections import deque
from pathlib import Path

from src.image_stitcher.camera import Camera
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.stitcher import ImageStitcher

queue = deque()
stitcher = ImageStitcher()  # or ImageStitcher(camera=Camera.load(path))

# Whatever receives photos from the drones appends to the queue...
queue.append(
    ImageMessage(
        lat=43.4723,
        lon=-80.5449,
        alt=25.0,
        img=rgb_array,
        roll_deg=1.2,
        pitch_deg=-0.8,
        yaw_deg=92.5,
    )
)

# ...and the stitcher drains it.
stitcher.process_queue(queue)
stitcher.save(Path("map.png"))  # map.png + map.json (GPS corners, m/px)
```

- `ImageStitcher.run(queue, stop_event, output_dir=..., save_every=20)` does
  the same in a loop on a worker thread until `stop_event` is set. Each save
  is a new numbered map (`map_001.png` + `.json`, `map_002...`), continuing
  after maps already in the folder so none are overwritten.
- Photos that can't be used are skipped and listed in `stitcher.rejections`
  as `(drone_id, image_id, reason)`.
- `files.load_photo_run(run_dir)` reads a photo run saved by the 3ds airside
  camera node. That node doesn't log roll, pitch or yaw yet, so until it adds
  an `attitude` entry those photos are rejected.

The first run needs internet: XFeat (about 6 MB, pinned commit) is
downloaded with `torch.hub` and cached in `~/.cache/torch/hub`. Run the demo
once on the groundside laptop before going to the field. It stitches a
synthetic 3-drone mission into `stitcher_demo/`, with `ground_truth.png` for
comparison:

```bash
cd 3ds_groundside
uv run python -m tests.image_stitcher.demo stitcher_demo --gps-noise 2
```

## How it works

For each new photo:

1. **Check.** Photos with no GPS or attitude, tilted, badly exposed, blurry
   (`quality.py`) or the wrong size for the calibration are rejected. The
   rest are undistorted with `camera_info.yaml` (`camera.py`), which also
   gives the ground resolution at each altitude.
2. **Match.** GPS picks nearby placed photos and
   [XFeat](https://github.com/verlab/accelerated_features) finds the same
   ground points in them. XFeat is not rotation invariant, so photos are
   turned north-up with their yaw first (a lossless quarter turn if within
   10 degrees of one).
3. **Verify.** RANSAC fits the shift, rotation and scale between photos. A
   match is rejected if it has too few points, its scale disagrees with the
   altitudes, or it puts the photo far from its GPS. A match that conflicts
   with the photo's other matches is left out of refinement.
4. **Place.** The photo joins the cluster of photos it lines up with,
   merging clusters it connects. A photo that matches nothing is placed at
   its GPS and snaps into place once neighbours arrive.
5. **Refine.** Every 10 photos and before drawing, all poses in a cluster
   are re-solved together (`refine.py`) so small errors don't pile up.

Each cluster is placed on the map with one best-fit rotation, scale and
shift against all its photos' GPS, so GPS errors average out. Until its
photos spread over `min_anchor_spread_m` in GPS and 4 x `gps_error_m` in the
images, a cluster uses the calibration for scale and yaw for rotation
instead. Yaw only lines photos up; final positions come from matches and GPS.

| File | Purpose |
|---|---|
| `message.py` | `ImageMessage`: what arrives on the queue |
| `stitcher.py` | `ImageStitcher`: matching, clustering, placing, rendering |
| `camera.py`, `camera_info.yaml` | Arducam 640 x 480 calibration, undistortion |
| `quality.py` | Rejecting blurry, badly exposed or tilted photos |
| `features.py` | XFeat wrapper, north-up feature extraction |
| `refine.py` | Joint least-squares refinement of photo poses |
| `canvas.py` | The growing map image and blending |
| `geo.py` | GPS / local metre / pixel geometry |
| `files.py` | Writing maps, reading 3ds airside photo runs |

Synthetic missions for the tests, the demo and the evaluation are in
`tests/image_stitcher/`.

## Accuracy

Simulated: synthetic 120 x 90 m missions at 25 m through the bundled
calibration (91 photos, 3 drones, shuffled arrival, yaw error sigma 5
degrees, altitude error sigma 0.3 m), 5 random fields each. One pixel is
about 4.5 cm of ground. Rerun with (about 3 minutes):

```bash
uv run python -m tests.image_stitcher.evaluate
```

| GPS error | Photos line up with each other | Map position vs truth | Connected map |
|---|---|---|---|
| 1 m | 4.0 to 7.3 cm | 0.162 to 0.307 m | 5 / 5 |
| 3 m | 4.3 to 8.3 cm | 0.472 to 0.935 m | 5 / 5 |

No photo was rejected. 0.12 to 0.19 s per photo on a laptop CPU across runs.
These missions fly east-west, so most photos get a lossless quarter turn;
other headings line up slightly less well.

## Configuration

`StitcherConfig` holds the tuning knobs, each documented in `stitcher.py`.
Most likely to need changing for real flights:

- `gps_error_m`: the real GPS accuracy.
- `min_inliers`: raise it if wrong matches appear over repetitive ground.
- `max_distance_m`: photos further than this from the first accepted photo
  are GPS glitches (400 m), so the survey must stay within it.
- `quality`: `max_tilt_deg` (5), `min_sharpness` (0.32),
  `max_clipped_fraction` (0.2). Picked on synthetic photos; check on real ones.

## Limitations

- Tested on synthetic images only; needs a run on real drone photos.
- Assumes flat ground and a straight-down camera. Small tilts are treated as
  straight down (at 25 m, 5 degrees moves the point below the drone 2.2 m).
- Sensor noise looks like detail, so the blur check is less reliable on
  noisy photos. Tune `min_sharpness` on the flight camera.
- `camera_info.yaml` must match the camera and resolution being flown. After
  recalibrating (`airside_prec/camera_calibration.py` on the
  `precision_landing` branch), replace it.
- Yaw is assumed clockwise from north for the image top; nothing detects if
  it isn't. A constant offset (camera mounted at an angle) still matches but
  turns clusters too small to fit to GPS the wrong way. Radians or the
  opposite direction can stop photos matching and split the map.
- Absolute position is only as good as the averaged GPS.
- Rendering redraws every photo and refinement grows with cluster size: fine
  for hundreds of photos, thousands would need tiling.

## Tests

```bash
uv run --extra dev pytest tests/unit             # fast, no XFeat
uv run --extra dev pytest tests/integration      # XFeat end to end, ~1 min
```
