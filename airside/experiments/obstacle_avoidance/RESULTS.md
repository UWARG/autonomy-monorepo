# Moving traffic results

Status on 2026-11-21: **draft; not cleared for live props-on flight.**

## High-speed turn check (2026-11-21)

The personal branch now includes the latest #232 changes. Live planning uses
WebSocket receive time, checks impossible position jumps, and guards turns with
a speed/age-dependent reachability disc. Reported traffic above the 5 m/s
test envelope holds and alerts; 15/20 m/s are unsupported stress cases.
The response allowance was raised from 3.5 to 4.5 s after one moving SITL run
measured a 1.93 s stop. It exceeds the 1.5 s stale threshold plus that stop;
repeat stop measurements before flight.

GNC passed 157 tests, capture/replay passed 7 tests, and the rebuilt Airside
image passed 79/79 ROS tests. Final-image SITL stress runs used independent
instances, a 5 m server keep-away, one setpoint publisher and rosbag recording:

| Scenario | Minimum extra margin | Outcome |
| --- | ---: | --- |
| 5 m/s sharp-turn loop | 14.982 m | Held/alerted; route incomplete |
| 15 m/s triangle | 14.966 m | Speed-envelope hold/alert; no progress |
| 20 m/s triangle | 14.974 m | Speed-envelope hold/alert; no progress |
| 15 m/s via listener | 14.966 m | 32 snapshots of index 2; speed hold/alert |

These are stress-only outcomes. They do not qualify 15/20 m/s traffic for
autonomous flight or show that the route can be completed around a fast loop.
The original server failure had no recording, so its root cause is not proven;
the previous straight-line-only prediction and fixed 5.75 m buffer were a
concrete risk for turns between 1 Hz updates.

With the final 4.5 s response allowance, three fresh 1.2 m/s moving-crossing
SITL runs reached the goal. Their minimum extra margins beyond the raw 5 m
keep-away were 5.880/5.787/5.788 m; measured peak speeds were
1.531/1.528/1.533 m/s. Each found one MAVROS setpoint publisher and saved a
rosbag. This is a local mock result, not a real server-track qualification.
The final-image two-aircraft crossing also reached the goal with 5.559 m
minimum extra margin and a 1.532 m/s peak.
The low-speed turn reached the goal with 16.034 m extra margin. Dropout held
on stale traffic and resumed after data returned; the first dropout harness
run falsely failed because it checked only the alert status, not the stored
`hold_reason`. The corrected harness run passed. The blocked-route test held
and alerted without retreating; pilot takeover stopped setpoints in LOITER
and did not replay an old target on returning to GUIDED. Each saved a rosbag.

The final-image listener-to-controller moving run also reached the goal:
57 index-2 snapshots, 5.754 m minimum extra margin, 1.544 m/s measured peak,
and one setpoint publisher. Its rosbag is saved locally.
The final-image full eight-waypoint mission-manager run completed one lap
with 266 index-2 snapshots, 6.138 m minimum extra margin, 1.559 m/s measured
peak, and one setpoint publisher. Its rosbag is saved locally.

A 360 s receive-only server capture had 360 moving snapshots at about 1 Hz:
index 0 reached 5 m/s; index 1 was stationary. It had no 15/20 m/s aircraft
and returned `NO_MOVING_ROUTE_MATCH` against the active eight-point route.
An independent 360 s receive-only capture on 2026-11-21 returned 359 moving
snapshots with the same indices and speed range. Its average interval was
1.000 s, with one 1.543 s gap, and it also returned `NO_MOVING_ROUTE_MATCH`.
No valid full-route server replay or simulated telemetry transmission occurred.
Docker ran out of host disk space during verification. After space was freed,
35.81 GB of unused Docker build cache was pruned; images, containers and test
artifacts were retained. The #181 static regression remains open.

The sections below are historical results from before the latest #232 sync.

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
