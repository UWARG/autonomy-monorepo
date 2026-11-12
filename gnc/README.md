# GNC Module

Guidance, Navigation, and Control.

## Development

```bash
warg run gnc setup     # uv sync
warg run gnc test      # uv run --extra dev pytest
warg run gnc lint      # uv run --extra dev ruff check .
```

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
| `decompose.py` | boustrophedon cell decomposition + cell ordering |
| `sweep.py` | per-cell coverage generation and orientation cost |
| `pathing.py` | orchestration: `plan_orthomosaic_path` |

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

## Lloyd's algorithm (even drone spacing)

Spreads a swarm evenly over a region with Lloyd's algorithm (Voronoi
relaxation). Each step gives every drone the part of the region closer to it
than to any other drone (its Voronoi cell, clipped to the region) and moves the
drone to that cell's centroid. Repeating this converges to a centroidal Voronoi
tessellation: every drone ends up in the middle of a similar share of the area.

### Usage

```python
from lloyd import coverage_cost, lloyd_relax, lloyd_step

positions = {1: Coordinate(lat, lon, alt), 2: Coordinate(lat, lon, alt), ...}
boundary = [Coordinate(lat, lon, 0.0), ...]  # ordered GPS vertices, >= 3

next_positions = lloyd_step(positions, boundary)  # {drone_id: Coordinate}

result = lloyd_relax(positions, boundary, max_iterations=100, tolerance_m=0.1)
result.positions    # final {drone_id: Coordinate}
result.iterations   # steps taken
result.converged    # True once no drone moved more than tolerance_m
result.history      # input positions, then the positions after every step

coverage_cost(positions, boundary)  # mean squared distance (m^2) to the nearest drone
```

`lloyd_step` is one step, for re-planning while drones fly; `lloyd_relax`
repeats it until the drones stop moving. Drone ids are kept and every drone
keeps its own altitude.

### Assumptions and limitations

- **Uniform importance.** Every part of the region counts the same; there is
  no density weighting yet.
- **Even spacing, not equal areas.** Lloyd's algorithm minimises the mean
  squared distance to the nearest drone. Drones end up evenly spaced, but cell
  areas can still differ (e.g. 6 drones in a square can settle with the
  largest cell up to about 1.8x the smallest, depending on where they start).
  Splitting work into equal shares is a different problem.
- **Convergence is gradual.** Big moves happen in the first few steps, then
  steps shrink; use `tolerance_m` and `max_iterations` to trade precision for
  time.
- **Planar.** Positions are projected to local east/north metres around the
  boundary centroid (same helpers as orthomosaic planning); altitude is ignored
  when splitting the region.
- **Concave regions.** A cell cut by the boundary can have its centroid outside
  it; the drone then goes to the nearest point of its own cell, so it always
  stays inside the region. Spacing in concave regions is even locally, but the
  result depends on the start (Lloyd's algorithm finds a local optimum).
- **Drones outside the region** are pulled back to its nearest point, kept
  at least 2 m from the other drones (closer only if the region is too small
  for that).
- **Supported boundaries** are the same as orthomosaic planning: simple
  polygons without holes. Two drones closer than 1 mm are rejected with
  `ValueError`, since there is no boundary between them.
- **No collision avoidance.** The output is a target per drone; flying there
  safely is up to the path planner.
