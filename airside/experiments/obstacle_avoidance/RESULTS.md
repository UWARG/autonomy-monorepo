# Moving traffic results

Status on 2026-11-19: **ready for code review; not ready for live props-on
flight testing.**

## Mission-manager moving SITL (2026-11-19)

The earlier listener-to-controller mock runs disabled the mission manager and
published a target without a waypoint path. New runs enabled the real mission
manager, used its spline path and completed one lap on each route:

- Three-point route: PASS. Four ordered targets including lap closure; path
  lengths 3, 5, 4, 3; 97 listener snapshots; aircraft 2 received; 4.523 m
  minimum measured margin beyond the raw 5 m keep-away; 1.540 m/s peak.
- Eight-point route fixture from #232: PASS. Nine ordered targets including lap
  closure; path lengths 3, 5, 5, 5, 5, 5, 5, 4, 3; 247 listener snapshots;
  aircraft 2 received; 4.678 m minimum margin; 1.568 m/s peak.

Both runs read back a 1.5 m/s FCU setting, found one MAVROS setpoint node,
and saw a temporary no-path hold/pilot alert before replanning and completing
the lap. Traffic age stayed under 1.0 s. The Airside image was rebuilt and
`colcon test` passed (70 tests, 0 failures). Rosbags and summaries are in
ignored `artifacts/engine-triangle-Qn1ONmuQ/` and
`artifacts/engine-eight-s6id62wP/`.

These tests rule out a general failure of #237's waypoint-path/16 m look-ahead
with this local crossing. They do not establish the cause of the reported
merged-copy failure, which has no known commit or rosbag. The eight-point
route is a test fixture; this branch's active route remains three-point. A
Git pull without an Airside image rebuild can also leave old installed code.
Real-track replay and props-off flight gates below remain open.

## Passed on the final image

- Airside image rebuilt successfully.
- GNC: 145 tests passed. Capture/replay: 7 tests passed. ROS: 70 tests passed.
- Ruff, Python compile and runner shell syntax checks passed.
- Mock moving crossing passed 3/3 at an FCU setting of 1.5 m/s. Measured peaks
  were 1.534-1.538 m/s; minimum margins outside the raw 5 m keep-away were
  5.577-5.680 m.
- Clear, multi-aircraft, turn, dropout, blocked-route and pilot-takeover SITL
  scenarios passed. Multi-aircraft kept 4.740 m extra margin. The blocked
  route held position and produced a pilot-takeover alert.
- Every SITL run found one MAVROS setpoint publisher.

## Listener-to-controller integration check (2026-11-17)

The rebuilt Airside image ran the real launch with its mission manager and
camera disabled. A local 1 Hz mock WebSocket fed the real traffic listener;
no competition-server token or telemetry sender was used. Three fresh SITL
moving-crossing runs passed and reached the goal. The listener filtered own
index 1 and published index 2 on `/aeac/traffic`. Measured peak speeds were
1.532-1.542 m/s. Minimum margins beyond the raw 5 m keep-away were
5.509-5.609 m. Each run found one setpoint publisher. The controller yielded,
briefly alerted for pilot takeover when its hold point was predicted to be
unsafe, then replanned and completed the route. One run briefly held on a
snapshot older than 1.5 s.

The listener disconnect/reconnect SITL run also passed: the controller held
on `AEAC_DISCONNECTED` and resumed after a healthy snapshot. Image rebuild,
ROS tests (70/70), GNC tests (145/145), and capture/replay tests (7/7) passed.
Rosbags and summaries are in ignored `artifacts/listener-*/` directories.

This verifies #237 alone, not an unidentified merged copy. Another obstacle
implementation publishes `/position_controller/obstacle`, while this controller
subscribes to `/aeac/traffic`; a mixed launch may fail closed, but its actual
source and rosbag are needed to establish the cause. No other remote branch
includes all of #237's commits. The PR base also has a newer eight-point route,
while this branch still uses the earlier three-point route. These results must not
be treated as qualification of the newer route.

## Real server evidence

A 720 s receive-only capture recorded 719 moving snapshots without sending
telemetry. It contained aircraft indices 0, 1 and 2. Source intervals ranged
from 0.000354 s to 4.330 s because updates sometimes arrived in batches or
with gaps; the 1.5 s stale hold is therefore exercised during replay.

The three preselected starts are 0, 180 and 360 s. The 2026-11-17 rebuilt-image
360 s run failed after completing 2/3 waypoints: the final leg stayed blocked
until timeout. It replayed 278 frames, used diagnostic own index 255, read back
1.5 m/s, measured a 1.533 m/s peak, and found one setpoint publisher. It stayed
outside the raw 5 m keep-away, but its 2.954 m extra margin was 0.046 m below
the 3 m target. The controller held on blocked or stale traffic and alerted for
pilot takeover. The run's rosbag and summary are in `artifacts/replay-dm4Fgy88/`.
The 0 and 180 s windows were not repeated because one failed window already
keeps the flight gate closed. Earlier runs are diagnostic only.

## Open flight gates

1. Resolve or approve the fixed-route blockage, then run the 0, 180 and 360 s
   real-track replays with the confirmed own-aircraft index. All must meet the
   route, spacing, speed, stale-hold and single-publisher criteria.
2. If the fixed route remains blocked, keep hold/replan/pilot alert and record
   the configuration as unavailable. Do not reduce the server keep-away to
   force route completion.
3. Complete the authorized props-off two-WebSocket check: own index,
   WGS84/AGL, approximately 1 Hz receive/send, and end-to-end delay.
4. Keep the first physical flight at 1 m/s. The 1.5 m/s result is SITL-only.

The PR should stay draft for flight-readiness purposes until these gates pass.
