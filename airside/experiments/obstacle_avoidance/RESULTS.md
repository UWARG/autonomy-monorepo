# Moving traffic readiness results

Test date: 2026-11-04. Branch base: #232 at
`66e86706a782520ce161fec412e87c623aab555a`.

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

- No real AEAC moving-traffic sample has been received. The local ignored
  `.env` did not contain a usable `AEAC_CONNECTION_TOKEN`, so the receive-only
  command stopped before opening a WebSocket. No telemetry was sent.
- A real capture must be replayed into SITL with its original timing and
  trajectory and pass the same measured-distance checks.
- The props-off two-WebSocket check still must verify the official endpoint,
  own aircraft index, WGS84/AGL fields, approximately 1 Hz receive/send rate,
  and end-to-end latency. Sending requires the team's explicit server test
  procedure and authorization.
- Physical first flight remains capped at 1 m/s. The 1.5 m/s result is a SITL
  result only.

Therefore this branch is ready for review and continued server qualification,
but it is **not yet approved for a live props-on AEAC flight test**.
