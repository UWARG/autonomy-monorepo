# Moving traffic readiness results

Test date: 2026-11-04 through 2026-11-05. Branch base: #232 at
`7c521c9596bb1007181b74e84b73d2474ba638a0`.

## Completed

- `gnc`: 145 tests passed.
- Airside `colcon test`: 60 tests passed, with no errors, failures or skips.
- Mock ArduCopter SITL at an FCU speed setting of 1.5 m/s:
  - three independent moving-crossing runs passed; peak measured speeds were
    1.518, 1.519 and 1.520 m/s, and minimum margins outside the server
    keep-away were 14.855, 14.857 and 14.869 m;
  - `multi` passed at 1.526 m/s with 13.963 m minimum margin;
  - `turn` passed at 1.527 m/s with 15.703 m minimum margin;
  - `dropout` held on stale traffic and resumed on fresh traffic;
  - `blocked` held within about 0.03 m, replanned, and emitted the pilot
    takeover alert after no safe path remained;
  - `pilot` stopped setpoints in LOITER, did not replay the old target when
    GUIDED resumed, and reached the new target only after a fresh command;
  - each run confirmed a single MAVROS setpoint publisher and produced an
    ignored rosbag plus JSON summary.
- The unchanged #181 imaginary-static-obstacle qualification was rerun in its
  own clean worktree: static obstacle 3/3 passed, and the clear and pilot
  takeover scenarios also passed (5/5 total).

The first moving-crossing implementation exposed a small measured raw
keep-away breach. That failed result was not accepted. A closest-approach
yield gate was added, followed by the three independent passing runs above.

## Not completed: hard flight-test gates

- A real capture must be matched to the intended test course, then replayed
  into SITL with its original timing and trajectory and pass the same
  measured-distance checks.
- The props-off two-WebSocket check still must verify the official endpoint,
  own aircraft index, WGS84/AGL fields, approximately 1 Hz receive/send rate,
  and end-to-end latency. Sending requires the team's explicit server test
  procedure and authorization.
- Physical first flight remains capped at 1 m/s. The 1.5 m/s result is a SITL
  result only.

Therefore this branch is ready for review and continued server qualification,
but it is **not yet approved for a live props-on AEAC flight test**.

## Route replay tooling check (2026-11-05)

- Added route matching and timing-preserving SITL replay for receive-only
  traffic JSONL. A generated crossing track intersected the first configured
  waypoint leg and was replayed on a fresh SITL boot.
- After rebasing onto #232's Catmull–Rom spline update, the generated-track
  smoke test passed again: 36 snapshots replayed at 0.999–1.009 s intervals,
  FCU speed readback 1.5 m/s, observed peak 1.513 m/s, minimum margin
  11.959 m outside the raw keep-away, one setpoint owner, and the leg goal
  reached after yielding to crossing traffic. The rebuilt Airside image also
  passed all 70 `colcon test` checks.
- A 180 s receive-only AEAC capture then succeeded without transmitting any
  telemetry: 184 healthy snapshots over 178.134 s (mean interval 0.973 s,
  median 0.987 s). Every snapshot contained moving indices 0 and 1. Both
  reported 15 m/s and a 30 m horizontal keep-away; index 0 was at 50 m AGL
  and index 1 at 80 m AGL. One interval was under 0.5 s and 18 exceeded
  1.2 s, so 1 Hz is an expected cadence rather than a hard latency bound.
- The checked-in Airside route is near Waterloo, while both captured tracks
  are near Medicine Hat. The matcher therefore correctly returned
  `NO_MOVING_ROUTE_MATCH`. The CONOPS says the prescribed 3–5 km lap course
  is supplied shortly before the event; its flight-boundary coordinates are
  not a substitute for that course. Sophie's test-course waypoints have not
  been found in this repository or its remote branches.
- Consequently the real server connection and moving traffic are verified,
  but the claim that Sophie's obstacle crosses the intended route is still
  **incomplete**. Three real-capture SITL trials cannot run until the matching
  course waypoint YAML (and, for live autonomy, our confirmed aircraft
  index) are available.
