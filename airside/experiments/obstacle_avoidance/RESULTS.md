# Moving traffic flight-test results

Status on 2026-11-07: **PR #237 is ready for code review, but live obstacle
avoidance is not ready for a props-on flight test.** This branch is based on
#232 at `7c521c9`.

## Verified

- Rebuilt Airside image: 70 `colcon test` checks passed. GNC and replay tests:
  151 passed. Capture/replay image tests: 7 passed.
- Mock SITL moving crossing passed 3/3. Clear, multi-aircraft, turn, dropout,
  blocked-route, pilot-takeover, and generated-track replay tests also passed.
- Mock runs used a 1.5 m/s FCU setting, measured at most 1.527 m/s, kept at
  least 13.963 m outside raw keep-away, used one setpoint publisher, and saved
  rosbags.
- The separate #181 static-obstacle regression was rerun with an isolated
  image: static passed 3/3 and the full readiness check passed 5/5.
- A new 240 s receive-only AEAC capture recorded 231 moving snapshots at
  0.918-1.110 s source intervals without sending telemetry. Indices 0, 1, and
  2 were present. Moving index 1 passed within 0.188 m of route segment 2.
- The replay tool now starts at the first route point and commands all three
  waypoints in order, including the closing leg. It no longer starts on the
  automatically selected encounter leg. Failed runs retain all safety metrics
  and a rosbag.

## Real-track full-route SITL: not passed

Three preselected capture windows at 0, 60, and 120 seconds were run with a
fresh SITL boot. Each window contained a real moving-route encounter. All
three runs timed out at waypoint 1/3 after repeated crossing-yield and pilot
takeover states; no run completed the route.

| Start | Result | Peak speed | Minimum margin outside raw keep-away | Traffic |
| ---: | --- | ---: | ---: | --- |
| 0 s | FAIL, waypoint 1/3 | 1.511 m/s | 5.767 m | Fresh |
| 60 s | FAIL, waypoint 1/3 | 1.505 m/s | 6.820 m | Fresh |
| 120 s | FAIL, waypoint 1/3 | 1.518 m/s | 3.846 m | Became stale after capture ended |

Every run found exactly one MAVROS setpoint publisher and remained outside
the 5 m raw horizontal keep-away. The fixed route and current traffic scenario
are therefore safe-hold evidence, not a successful avoidance qualification.
The runs used diagnostic own index 255; this does not establish the real UAV
index. Artifacts are stored locally under ignored `artifacts/replay-*` paths.

## Before live props-on flight

1. Do not reduce raw keep-away to force a pass. The fixed route needs an
   approved traffic or route change before goal-completion qualification can
   pass; until then, keep hold/replan/pilot takeover behavior.
2. Confirm the real aircraft index and run the authorized props-off check for
   both WebSocket connections, WGS84/AGL fields, approximately 1 Hz send and
   receive rates, and end-to-end latency.
3. Keep the first physical flight at 1 m/s. The 1.5 m/s result is SITL-only.

The PR should remain draft for flight-readiness purposes. It can still receive
code review while the external flight gates remain open.
