# Obstacle-avoidance test status

Status on 2026-11-27: **draft; ready for others to test in SITL.**

## Implemented

- Live AEAC traffic is consumed at about 1 Hz and converted to predicted local
  exclusion areas. Server keep-away is never reduced.
- Traffic older than 1.5 s, invalid motion, unsupported speed, no path, or a
  hold-point conflict stops autonomous progress and alerts the pilot.
- Replanning retains a safe detour instead of switching targets every update.
  A stalled route tries a meaningfully different safe heading.
- A blocked required waypoint can move up to 20 m outward from the route
  polygon, but only inside an enabled flight boundary. The replacement must be
  reached within 3 m and cannot be skipped.
- The mission manager no longer reports a timed-out or blocked waypoint as
  complete.
- The independent 1 Hz telemetry sender validates fresh MAVROS WGS84 and AGL
  data. It is opt-in and remains disabled during receive-only tests.

## Final local verification

- Docker image: built from the current branch.
- GNC: 188 tests passed.
- ROS/colcon: 96 tests passed, with no failures or skipped tests.
- Capture, configuration and replay tools: 10 tests passed.
- Full eight-waypoint mock mission: PASS on the final image. The blocked
  waypoint moved outward, the replacement was reached at 2.76 m, all waypoints
  completed, and only `position_controller` published MAVROS setpoints.
- That mission measured 1.55 m/s peak ownship speed, 1.01 s maximum traffic
  age, and at least 4.20 m clearance beyond the server keep-away cylinder.
- Listener-to-controller moving traffic also passed on the final architecture,
  including own-aircraft filtering and disconnect handling.
- Active WRESTRC triangle: two full mock SITL laps passed with home
  `43.4338803,-80.5775947,330,0`. A 1.2 m/s mock aircraft crossed a travel
  leg at 1 Hz; the controller reported `YIELD_CROSSING`. In the final-image
  run, peak speed was 1.562 m/s and the closest pass stayed 10.13 m beyond its 5 m
  keep-away radius. Only `position_controller` published MAVROS setpoints.
- In this isolated run, `/position_controller/target` had only
  `airside_interfaces/msg/PositionTarget`. Rosbag recorded it without the
  reported multiple-type error. The failing external ROS graph is still
  needed to identify its second type and publisher.
- Sharp-turn and 15/20 m/s scenarios safely hold and alert when motion cannot
  be bounded. These are stress tests, not supported props-on traffic speeds.

The final full-mission artifact is ignored locally at
`artifacts/engine-replace-CwbDm3Xt/`; it includes the summary, logs and rosbag.
The final-image WRESTRC run is at `artifacts/engine-triangle-KaqN1lcV/`.

## Receive-only server replay

A receive-only capture contained 298 WRESTRC snapshots near 1 Hz, with one
7.27 s gap. No simulated ownship telemetry was sent.

Three independent full mission-manager replays used an earlier WRESTRC triangle
and different capture start times. All three held and alerted without entering
the server keep-away area, but none completed the route. The selected ownship
start was already inside the conservative expanded traffic reachability area;
the last replay also exhausted the capture. These runs do not qualify the new
active triangle and are safe failure evidence, not an autonomous-avoidance pass.

## Open flight gates

Before live props-on use:

1. Load an approved WGS84 flight boundary and matching FCU fence.
2. Confirm the real own-aircraft index; do not use the diagnostic replay index.
3. Capture a complete same-route traffic window and pass three fresh full-route
   replays, or record the route as unavailable when traffic blocks it.
4. With authorization and props off, verify the receive and telemetry sockets,
   about 1 Hz traffic and telemetry, WGS84/AGL values, and end-to-end delay.
5. Keep the first physical flight limited to 1 m/s with a safety pilot ready to
   take over.

Until all gates pass, the PR stays draft and props-on is not cleared. A mock
pass does not replace the same-route server replay or the props-off check.
