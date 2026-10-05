# GNC Module

Guidance, Navigation, and Control.

The first implemented component is **`orthomosaic`**: a pure-Python orthomosaic
lawn-mower coverage planner. Given a camera, a target ground resolution, and a
GPS boundary of an area to map, it returns the GPS coordinates where a nadir
camera drone should fly and take pictures so the photos can be stitched into a
complete orthomosaic.

This is coverage **planning** only. Obstacle avoidance, actual flight control,
and image stitching are separate concerns outside this module.

## Orthomosaic planning

### Usage

```python
from orthomosaic import CameraSpec, ScanRequest, plan_orthomosaic_path

camera = CameraSpec(
    image_width_px=1280,
    image_height_px=720,
    horizontal_fov_rad=1.396,   # radians (~80 deg)
    vertical_fov_rad=0.960,     # radians (~55 deg)
)

request = ScanRequest(
    boundary=[Coordinate(lat, lon, 0.0), ...],  # ordered GPS vertices, >= 3
    camera=camera,
    target_gsd_m=0.02,          # 2 cm/pixel desired ground resolution
    forward_overlap=0.75,       # along-track photo overlap
    side_overlap=0.65,          # cross-track (between-lines) overlap
    start=Coordinate(lat, lon, 0.0),  # optional; where the drone begins
)

plan = plan_orthomosaic_path(request)

plan.altitude_agl_m        # derived flight altitude (an output, not an input)
plan.capture_waypoints     # GPS Coordinates where photos are taken
plan.path_waypoints        # full flyable path (captures + pass corners + start)
plan.sweep_angle_deg       # chosen sweep orientation
plan.estimated_path_length_m, plan.estimated_turn_count, plan.photo_count
```

### What it computes

1. **Altitude** — derived from the target GSD, camera FOV, and image size. The
   limiting axis exactly meets the target resolution; the other axis ends up
   finer.
2. **Footprint & spacing** — ground swath/frame from FOV and altitude; line
   spacing `= swath·(1 − side_overlap)`, capture spacing
   `= frame·(1 − forward_overlap)`.
3. **Orientation** — candidate sweep angles (polygon edge directions + a
   `angle_step_deg` grid) are each decomposed, swept, and scored with
   `cost = straight_length + turn_penalty·turns`; the cheapest wins.
4. **Boustrophedon decomposition** — concave regions are split into monotone
   cells by cutting at critical heights, so each cell is covered by clean
   back-and-forth passes.
5. **Coverage** — each pass covers the exact extent of the region inside its
   swath band; passes are placed so their bands tile the region with overlap,
   guaranteeing complete coverage.
6. **Assembly** — cells are ordered (nearest-neighbour from `start`), connected
   by straight transits, and every waypoint is converted back to GPS with the
   derived altitude attached.

### Package layout

| File | Responsibility |
|---|---|
| `models.py` | `CameraSpec`, `ScanRequest`, `ScanPlan` value objects |
| `footprint.py` | altitude, footprint, GSD, and spacing math |
| `coordinates.py` | GPS↔local east/north conversion and rotation |
| `decompose.py` | boustrophedon cell decomposition + cell ordering |
| `sweep.py` | per-cell coverage generation and orientation cost |
| `pathing.py` | orchestration: `plan_orthomosaic_path` |

### Development

```bash
warg run gnc setup     # uv sync
warg run gnc test      # uv run --extra dev pytest
warg run gnc lint      # uv run --extra dev ruff check .
```

The module depends on `utils` (shared types/waypoint math) and `shapely`
(geometry). Like the rest of the repo, `utils` is imported from the monorepo
root, so runtimes must put the monorepo root on `sys.path` (as airside's
container already does).

### Assumptions and limitations

- **Nadir camera over flat ground.** Footprint/GSD math assumes the camera
  points straight down and terrain is flat.
- **Camera mounting.** The image-width axis is assumed to be perpendicular to
  the flight direction (i.e. `hfov` drives cross-track spacing). Swap swath and
  frame if the camera is mounted differently.
- **Supported boundaries.** Simple polygons without holes. Self-intersecting,
  degenerate, or holed boundaries are rejected with `ValueError`.
- **Row-to-row overlap at sloped edges.** Coverage is guaranteed complete, but
  adjacent passes can have different lengths, so photo overlap between rows at
  sloped cell edges is not guaranteed — a stitching-quality concern, not a
  coverage gap.
- **Orientation search** is exact for convex cells (edge angles include the
  pass-count-optimal direction) and a step-resolution heuristic for concave
  regions.
- **Turn penalty** defaults to `π·line_spacing` (twice the ideal 180-degree
  turn arc) and is configurable via `ScanRequest.turn_penalty_m`.
- **Spherical Earth model.** Local east/north projection uses the IUGG mean
  Earth radius; accurate to sub-meter over scan-scale regions.