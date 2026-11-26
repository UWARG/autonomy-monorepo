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

avoider = ObstacleAvoider(AvoidanceConfig(
    horizontal_speed_mps=2.0,
    # Optional allowed flight area, as ordered GPS polygon vertices.
    flight_boundary=(
        Coordinate(43.0, -80.0, 0.0),
        Coordinate(43.0, -79.999, 0.0),
        Coordinate(43.001, -79.999, 0.0),
        Coordinate(43.001, -80.0, 0.0),
    ),
))

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

1. **Traffic prediction** — live mode includes every other aircraft regardless
   of altitude. Its circle radius is the server keep-away plus 5.75 m for
   moving traffic, or no additional buffer for stationary traffic. Positions
   are advanced by the supplied snapshot age and swept for 3 s. The lower-level
   conversion API can still use a vertical band and raw radius with
   `live_buffer=False`.
2. **Local planning** — `BendyRuler2D` searches candidate headings for one
   lookahead leg, up to 16 m, and checks its geometric clearance against the
   predicted circles. If configured, a GPS flight-boundary polygon filters
   candidate segments before route scoring. Paths starting outside the
   boundary may move closer to it or safely re-enter, but may not cross through
   it and end back outside. Each geometrically feasible candidate is also checked
   for closest approach against traffic moving at constant velocity during the
   leg's estimated travel time (capped at 20 s). Conflicting candidates are
   rejected before selection. If dynamic conflicts eliminate all candidates,
   the avoider checks whether holding is safe; it yields if so and requests
   pilot takeover otherwise. Geometric blockage still produces a no-path hold.
   If the current position is already inside an aircraft's expanded keep-away,
   an egress mode allows only candidates that immediately increase separation
   from every current conflict and remain clear of other aircraft. If no such
   candidate exists, the avoider holds and requests pilot takeover.
3. **Command** — the first lookahead point is returned as the next waypoint;
   the velocity fields are capped at `horizontal_speed_mps` and
   `vertical_speed_mps`.

### Safety contract

- Every failure is a zero-velocity `NO_PATH` hold with a reason: stale or
  future traffic, invalid or duplicate aircraft, too many predicted circles,
  invalid position, or no safe path. `EGRESSING_CONFLICT` means a safe
  separation-increasing recovery leg is being flown. If no such leg exists,
  `HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED` means the modeled hold point is
  unsafe; the pilot must take over.
- A returned leg meets the geometric clearance margin and the modeled
  constant-velocity closest-approach check. This depends on the supplied
  traffic age and does not model acceleration or other flight dynamics.
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
| `planner.py` | `BendyRuler2D` single-leg lookahead search |
| `clearance.py`, `geometry.py` | clearance queries and 2D geometry |
| `avoidance.py` | orchestration: `ObstacleAvoider.plan` |
