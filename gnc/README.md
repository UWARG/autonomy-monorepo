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

## Obstacle avoidance

### Usage

```python
from obstacle_avoidance import (
    AvoidanceConfig, AvoidanceRequest, ObstacleAvoider, TrafficAircraft,
)

avoider = ObstacleAvoider(AvoidanceConfig(horizontal_speed_mps=2.0))

plan = avoider.plan(AvoidanceRequest(
    position=Coordinate(lat, lon, rel_alt_m),   # where we are now
    goal=Coordinate(lat, lon, rel_alt_m),       # waypoint we are flying to
    traffic=(TrafficAircraft(
        aircraft_index=3, lat=..., lon=..., altitude_agl_m=...,
        speed_mps=..., heading_deg=...,
        horizontal_keep_away_m=..., vertical_keep_away_m=...,
    ), ...),
    traffic_age_s=0.4,                          # how old the traffic sample is
))

plan.status                # PATH_FOUND or NO_PATH
plan.reason                # why it is holding, e.g. "BLOCKED", "STALE_TRAFFIC"
plan.east_mps, plan.north_mps, plan.up_mps   # ENU velocity to fly this cycle
plan.next_waypoint         # the planner's next safe point, as a Coordinate
plan.goal_distance_m, plan.goal_reached, plan.minimum_clearance_m
```

Call `plan` once per control cycle with the same `ObstacleAvoider` (it keeps
heading hysteresis between calls) and `reset()` it when control is interrupted.

### What it computes

1. **Traffic prediction** — each aircraft whose vertical keep-away band
   overlaps our current-to-goal altitude band becomes a circle of its
   horizontal keep-away radius, moved forward by `traffic_age_s` and swept
   along its heading for `prediction_horizon_s`.
2. **Local planning** — `BendyRuler2D` searches candidate headings for a
   two-segment lookahead path that keeps `clearance_margin_m` from every
   circle, preferring the goal direction and the previous detour.
3. **Command** — the first lookahead point becomes a horizontal velocity
   capped at `horizontal_speed_mps`; altitude error becomes a vertical velocity
   capped at `vertical_speed_mps`.

### Safety contract

- Every failure is a zero-velocity `NO_PATH` hold with a reason: stale or
  future traffic, invalid or duplicate aircraft, too many predicted circles,
  invalid position, or no safe path.
- A returned path has both lookahead segments at or above the clearance margin.
- Candidate ordering and ties are deterministic.

The lower-level planner (`BendyRuler2D`, `PlanRequest`, `ObstacleSnapshot`,
`CircleObstacle`) and `traffic_to_obstacles` are exported too, for callers that
work in their own local frame. This is an original MIT-licensed implementation;
ArduPilot BendyRuler is a behavioral reference only, no GPL source is copied.

### Package layout

| File | Responsibility |
|---|---|
| `models.py` | `TrafficAircraft`, `AvoidanceRequest`, `AvoidancePlan` and planner value objects |
| `traffic.py` | traffic aircraft → predicted ENU exclusion circles |
| `planner.py` | `BendyRuler2D` two-segment lookahead search |
| `clearance.py`, `geometry.py` | clearance queries and 2D geometry |
| `avoidance.py` | orchestration: `ObstacleAvoider.plan` |