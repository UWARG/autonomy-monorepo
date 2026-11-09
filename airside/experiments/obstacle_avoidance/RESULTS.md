# Moving traffic results

Status on 2026-11-08: **ready for code review; not ready for live props-on
flight testing.**

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

## Real server evidence

A 720 s receive-only capture recorded 719 moving snapshots without sending
telemetry. It contained aircraft indices 0, 1 and 2. Source intervals ranged
from 0.000354 s to 4.330 s because updates sometimes arrived in batches or
with gaps; the 1.5 s stale hold is therefore exercised during replay.

The three preselected starts are 0, 180 and 360 s. The final-image 360 s run
failed after completing 2/3 waypoints: the route stayed blocked until timeout.
It used diagnostic own index 255, read back 1.5 m/s, measured a 1.527 m/s peak,
found one setpoint publisher, and stayed outside the raw 5 m keep-away. Its
2.973 m extra margin was also 0.027 m below the 3 m target. The 0 and 180 s
final-image runs were not repeated because one failed window already keeps the
flight gate closed. Earlier experimental-image runs are diagnostic only.

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
