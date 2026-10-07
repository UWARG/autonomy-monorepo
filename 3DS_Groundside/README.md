# 3DS Groundside

Groundside operator tooling for 3DS autonomy. The benchmark harness generates
repeatable target polygons and scores cumulative mapping footprints.

## Benchmark workflow

Run from `3DS_Groundside` after `uv sync --extra dev`:

```bash
uv run python -m src.benchmark generate --zone examples/zone.yaml --output suite.yaml --count 10 --seed 42
uv run python -m src.benchmark listen --suite suite.yaml
# Or replay a producer's JSONL recording:
uv run python -m src.benchmark score --suite suite.yaml --updates updates.jsonl
```

A zone file is a YAML list of `[x, y]` vertices in local meters. All target and
footprint coordinates must use the same origin and axes. Geographic latitude /
longitude must be projected to this frame upstream. Polygons may be concave;
self-intersections, zero area and nonfinite coordinates are rejected. Generated
targets are convex, distinct and completely inside the zone. A fixed seed repeats
an identical suite for comparing algorithms. Generation has a bounded attempt
budget and reports failure for zones too narrow to sample effectively.

## Provisional scenario contract (version 1)

```yaml
version: 1
coordinate_frame: local_meters
seed: 42
zone: [[0, 0], [100, 0], [100, 100], [0, 100]]
scenarios:
  - id: scenario-0001
    polygon: [[10, 10], [50, 10], [50, 50], [10, 50]]
```

The polygon is a mapping target, not a flight path. No altitude, waypoint planning
or aircraft control is supplied by this harness. The repository currently has no
groundstation polygon import format or orthomosaic progress producer. This YAML
contract and the following WebSocket contract are provisional integration points
for issue #196, not claims of compatibility with an existing groundstation.

## Producer contract

Connect to `ws://127.0.0.1:8765` and send one JSON object per WebSocket message:

```json
{"scenario_id":"scenario-0001","elapsed_s":2.5,"footprints":[[[10,10],[30,10],[30,50],[10,50]]]}
```

Each footprint is the boundary of valid mapped imagery, supplied by the
orthomosaic program. Footprints accumulate; previously reported areas are never
removed. Repeated or overlapping footprints are counted once. Empty footprints
allow a time-only update. `elapsed_s` is producer-measured time since run start,
finite, nonnegative and nondecreasing per scenario. An independent run starts
with each new connection; reconnecting resets scores. Scenario state is isolated.
Messages are limited to 1 MiB. Invalid messages receive `{"error":"..."}` without
changing state; the connection remains usable. Scores are returned as JSON and
also printed as JSONL on stdout, which can be redirected to a results file.
Use `--host` and `--port` to configure the bind address. The default is loopback;
the listener has no authentication or encryption.

Scores include target, covered, missed and outside areas in square meters:

- `coverage = covered_area / target_area` (0 to 1).
- `iou = covered_area / (target_area + outside_area)` (0 to 1).

IoU means intersection over union: the overlap divided by the combined area.
Full accurate coverage gives 1; missing the entire target gives 0. Mapping outside
the target reduces IoU even when coverage is complete. Elapsed time enables
coverage-versus-time comparisons, without mixing a speed weight into the score.
These are geometric completeness metrics, not image sharpness, alignment,
geolocation accuracy or 3D reconstruction quality metrics.

## Verification

```bash
uv run --extra dev pytest
uv run --extra dev ruff check .
```

Tests cover concave-zone containment, uniqueness and seed reproducibility, YAML
round trips, overlapping footprints, outside-area penalties, invalid geometry,
atomic invalid updates, elapsed-time ordering, CLI replay and a real local
WebSocket exchange. No drone, ROS stack or orthomosaic service is required.
