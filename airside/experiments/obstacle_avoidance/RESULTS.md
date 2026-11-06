# Moving traffic readiness results

Test date: 2026-11-04 through 2026-11-06. Branch base: #232 at
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

- The new Waterloo server capture matches the checked-in Airside route, but
  its first timing-preserving SITL replay did not reach the route-leg goal.
  Resolve that result and pass three independent real-track SITL trials with
  the same measured-distance checks before live props-on use.
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
- The checked-in Airside route is near Waterloo, while both tracks in this
  first capture were near Medicine Hat. The matcher therefore correctly
  returned `NO_MOVING_ROUTE_MATCH`. Later server-configuration screenshots
  supplied the Waterloo aircraft geometry described below.

## Sophie aircraft 2 check (2026-11-06)

- The server configuration screenshots label the new blue obstacle Aircraft 2,
  moving at 5 m/s and 10 m AGL between
  `(43.433992, -80.577849)` and `(43.433916, -80.577873)`. Against the
  checked-in three-point Waterloo lap, its transit segment comes within
  0.037 m of route segment 0, so the configured paths do cross.
- A new authenticated 60 s receive-only capture produced 57 healthy moving
  snapshots at 0.842–1.218 s intervals. Every snapshot contained only
  aircraft 0 and 1 from the Medicine Hat scenario; aircraft 2 appeared zero
  times and no parser problem was reported. The client accepts indices
  0–255 and does not filter index 2. This was the state before Sophie switched
  the active server scenario; see the fresh capture below.
- A screenshot-derived 1 Hz mock was replayed using the exact aircraft-2
  transit points, speed and altitude. Its keep-away is not visible in the
  screenshot, so 30 m and 5 m were tested only as explicitly provisional
  values. At 30 m, the route-leg start is already inside the raw exclusion
  zone and the controller correctly held. At 5 m, the conservative released
  code stayed 11.805 m outside the raw keep-away but did not complete the leg;
  it yielded to the continuously crossing traffic and requested pilot
  takeover.
- An experimental reduction of the live buffer briefly entered the
  provisional 5 m raw keep-away by 0.027 m. That result was rejected and the
  buffer change was fully reverted. No unqualified safety relaxation was
  committed.

## Waterloo server scenario after site switch (2026-11-06)

- A fresh 60 s authenticated, receive-only capture contained 60 valid traffic
  snapshots at 0.953–1.031 s intervals, with no parser problems. Every frame
  contained WebSocket indices 0, 1 and 2. Coordinates matched the screenshots
  exactly: index 0 follows the 50 m AGL three-point triangle, index 1 follows
  the newly added Aircraft 2 two-point track at 10 m AGL and 5 m/s, and index 2
  is the newly added Aircraft 3 stationary hold at 10 m AGL and 0 m/s. The UI
  aircraft numbers therefore differ from the zero-based WebSocket indices.
  All three reported 5 m horizontal and 30 m vertical keep-away.
- The route matcher found moving WebSocket index 1 within 0.188 m of segment 2
  of the checked-in 10 m Airside route. The capture had enough original timing
  for a route-leg replay. The screenshot's 50 m triangle is index 0's server
  traffic route, not the checked-in Airside route.
- One diagnostic ArduCopter SITL replay used an unassigned test-only own index
  of 255 so all three simulated aircraft remained in the feed. It failed to
  reach the route-leg goal: the controller reported
  `HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED`, emitted pilot takeover alerts, and
  later held on `STALE_TRAFFIC` after the finite capture ended. The minimum
  measured margin outside the raw 5 m keep-away was 6.816 m. A rosbag and
  summary were saved under the ignored `artifacts/replay-WMfRPwH5/` directory.
  This is a failed goal-completion test, not a qualified avoidance pass; the
  test-only index does not confirm our real aircraft identity.

The new server feed resolves the earlier missing-aircraft and unknown-radius
questions. The real-track replay failure, three independent passing trials,
confirmed own index, and props-off bidirectional checks remain flight gates.
