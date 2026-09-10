# Obstacle avoidance

WARG-owned local obstacle avoidance for issue 96. Version 1 is a deterministic
2D, two-segment lookahead planner. It accepts a sensor-neutral obstacle
snapshot and returns either a safe temporary waypoint (`PATH_FOUND`) or an
explicit hold requirement (`NO_PATH`).

The package contains no ROS, MAVLink, camera, or flight-controller imports.
Those adapters live outside the planner so a future 3D search can reuse the
same freshness, result, and integration contracts.

## Run

```bash
uv run --group dev pytest
uv run --group dev ruff check .
uv run python -m obstacle_avoidance
```

The command-line demo runs the pure planner against a finite wall. The
ArduCopter qualification scenario is in
`airside/scripts/avoidance/avoidance_demo.py`.

## Safety contract

- Stale or unhealthy obstacle data returns `NO_PATH`; callers must command
  zero velocity or hold.
- A returned path has both lookahead segments at or above the configured
  clearance margin.
- Candidate ordering and symmetric ties are deterministic.
- Hysteresis keeps a still-safe prior detour unless a new route materially
  improves goal alignment or clearance.

This is an original MIT-licensed implementation. ArduPilot BendyRuler is a
behavioral reference only; no GPL source is copied or translated.
