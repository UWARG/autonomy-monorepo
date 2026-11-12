# Image Stitcher

Groundside orthomosaic stitcher for 3DS ([#179](https://github.com/UWARG/autonomy-monorepo/issues/179)).
It takes photos from a fleet of drones, arriving in any order, and builds one
top-down map of the whole area.

Each photo needs its GPS position (lat, lon, alt) and the drone's roll, pitch
and yaw (degrees clockwise from north that the top of the image faces, either
-180 to 180 or 0 to 360). Photos are undistorted with the camera calibration
in `camera_info.yaml`, which also gives the ground resolution at each altitude.

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

`ImageStitcher.run(queue, stop_event, output_dir=..., save_every=20)` does
the same in a loop on a worker thread until `stop_event` is set. Every save is
a new numbered map (`map_001.png` + `map_001.json`, `map_002...`), continuing
after any maps already in the folder, so earlier maps are never overwritten.
`files.load_photo_run(run_dir)` reads a photo run saved by the 3ds airside
camera node (`photos.jsonl` + PNGs). The camera node doesn't log roll, pitch
or yaw yet, so until it adds an `attitude` entry those photos are rejected.

Photos that can't be used are skipped and listed in
`stitcher.rejections` as `(drone_id, image_id, reason)`.

The first run needs internet: XFeat is downloaded from its GitHub repo with
`torch.hub` (pinned commit, about 6 MB) and cached in `~/.cache/torch/hub`.
Run the demo once on the groundside laptop before going to the field.

Demo on a synthetic 3-drone mission (writes the numbered maps to
`stitcher_demo/`, the last one being the full map, and `ground_truth.png` for
comparison):

```bash
cd 3ds_groundside
uv run python -m src.image_stitcher.demo stitcher_demo --gps-noise 2
```

## How it works

For each new photo:

0. **Check and undistort.** Photos are rejected, and never placed, if they
   have no GPS or roll/pitch/yaw, are tilted, badly exposed or blurry
   (`quality.py`), or don't match the camera calibration's size. Good
   photos are undistorted with the calibration (`camera.py`).
1. **Find neighbours.** GPS picks the already-placed photos close enough to
   overlap it.
2. **Match features.** [XFeat](https://github.com/verlab/accelerated_features)
   finds the same ground points in the new photo and each neighbour. XFeat
   is not rotation invariant and lawn mower paths flip 180 degrees every row,
   so each photo is turned north-up with its yaw before extraction (a
   lossless quarter turn if its yaw is within 10 degrees of one). Yaw is only
   used to line photos up; their final positions come from the matches and
   GPS.
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
instead of making photos disagree with each other. A cluster only gets this
fit once both its GPS positions and its images show the photos are spread out
(4 x `gps_error_m`, 20 m by default, in the images). Otherwise GPS noise could
blow up the scale, so it uses the camera calibration for scale and yaw for
rotation. Overlaps are blended with weights that fade towards each photo's
edges, hiding seams.

| File | Purpose |
|---|---|
| `message.py` | `ImageMessage`: what arrives on the queue |
| `camera.py` | Camera calibration: undistortion, ground resolution |
| `quality.py` | Rejecting blurry, badly exposed or tilted photos |
| `camera_info.yaml` | Arducam 640 x 480 calibration (ROS camera_info format) |
| `stitcher.py` | `ImageStitcher`: matching, clustering, placing, rendering |
| `features.py` | XFeat wrapper, north-up feature extraction |
| `refine.py` | Joint least-squares refinement of photo poses |
| `canvas.py` | The growing map image and blending |
| `geo.py` | GPS / local metre / pixel geometry |
| `files.py` | Writing maps, reading 3ds airside photo runs |
| `synthetic.py` | Fake missions with known ground truth, for tests and the demo |
| `evaluate.py` | Accuracy on synthetic missions (the figures below) |

## Accuracy

Simulated: synthetic 120 x 90 m missions flown at 25 m through the bundled
camera calibration (91 photos, 3 drones, shuffled arrival, yaw error sigma
5 degrees, altitude error sigma 0.3 m), 5 random fields each. One pixel covers
about 4.5 cm of ground. Rerun with (about 3 minutes):

```bash
uv run python -m src.image_stitcher.evaluate
```

| GPS error | Photos line up with each other | Map position vs truth | Connected map |
|---|---|---|---|
| 1 m | 4.0 to 7.3 cm | 0.162 to 0.307 m | 5 / 5 |
| 3 m | 4.3 to 8.3 cm | 0.472 to 0.935 m | 5 / 5 |

No photo was rejected by the quality checks. 0.12 to 0.19 s per photo on a
laptop CPU across runs. These missions fly east-west, so most photos get a lossless quarter
turn. A yaw more than 10 degrees from a quarter turn gets a full rotation,
which lines photos up slightly less well.

## Configuration

`StitcherConfig` holds the tuning knobs, each documented in `stitcher.py`.
The ones most likely to need changing for real flights:

- `gps_error_m`: set to the real GPS accuracy. It widens the neighbour search
  and sets how far photos must spread before a cluster is fitted to GPS.
- `min_inliers`: raise it if wrong matches appear over repetitive ground.
- `resolution_m`: fixed output metres per pixel (default: the photos' own).
- `max_distance_m`: photos whose GPS is further than this from the first
  accepted photo are rejected as glitches (400 m), so the whole survey must
  lie within 400 m of where it starts.
- `quality`: rejection limits (`QualityLimits`): `max_tilt_deg` (5, roll and
  pitch combined), `min_sharpness` (0.32) and `max_clipped_fraction` (0.2).
  The image limits were picked on synthetic photos; check them on real ones.

## Limitations / future work

- Assumes flat ground and a straight-down camera. Hills and buildings will
  show small misalignments. Photos tilted past `max_tilt_deg` are rejected
  rather than corrected, and smaller tilts are treated as straight down: at
  25 m a 5 degree tilt moves the ground point below the drone by 2.2 m.
  Correcting it needs the camera's mounting on the drone.
- The blur check is less reliable on noisy photos: sensor noise looks like
  fine detail. A median filter removes most of it, but in simulation even
  mild noise (sigma 2 grey levels) can let a 5 px motion-blurred photo pass.
  Tune `min_sharpness` on real photos from the flight camera.
- The calibration file must match the camera and resolution being flown.
  After recalibrating (`airside_prec/camera_calibration.py` on the
  `precision_landing` branch), replace `camera_info.yaml`.
- A photo that overlaps nothing is placed by GPS and its yaw alone. It snaps
  into place once neighbours arrive.
- Yaw is assumed to be degrees clockwise from north for the top of the image,
  and nothing detects if it isn't. A constant offset (a camera mounted at an
  angle) still matches but turns clusters too small to fit to GPS the wrong
  way. Radians or the opposite direction can stop photos matching, which
  splits the map into pieces.
- The map's absolute position is only as good as the averaged GPS (see
  Accuracy). Ground control points would be needed for better.
- Rendering redraws every photo, and refinement cost grows with cluster size.
  Fine for hundreds of photos; thousands would need tiling.
- Tested on synthetic images only; needs a run on real drone photos.

## Tests

```bash
uv run --extra dev pytest tests/unit             # fast, no XFeat
uv run --extra dev pytest tests/integration      # XFeat end to end, ~1 min
```
