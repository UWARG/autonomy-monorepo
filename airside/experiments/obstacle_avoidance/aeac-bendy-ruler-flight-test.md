# AEAC server → BendyRuler2D flight-test path

This is the primary live-traffic path on PR #181. It is **not** the synthetic
stationary-obstacle ground test, and it is not Honzik's A* fallback branch.

| Mode | Traffic input | Planner | Launch |
|---|---|---|---|
| Live AEAC primary | `/aeac/live_traffic` from `comms/traffic_listener` | #181 BendyRuler2D lapping | `aeac_bendy_ruler.launch.py` |
| Stationary ground test | synthetic `/aeac/traffic` | #181 BendyRuler2D lapping | `static_obstacle_flight_test.launch.py` |
| Honzik fallback | `/position_controller/obstacle` | #203 A* visibility graph | Separate #203 branch, not combined with #181 launches |

The untouched #203 fallback currently has the traffic listener but no onboard
AEAC telemetry sender. It therefore needs a separately validated sender before
it can meet the CONOPS two-way server requirement; do not assume switching
branches alone supplies both WebSocket connections.

The live listener also publishes one `Obstacle.msg` per other aircraft on
`/position_controller/obstacle` and an atomic
`/position_controller/obstacle_snapshot` status for the fallback interface.
Its `/aeac/live_traffic` output is the *same complete server event*, mapped to
`TrafficSnapshot` for BendyRuler2D. The listener excludes the configured own
`aircraftIndex`; the planner converts WGS84 latitude/longitude to local metres
only inside its traffic conversion. The sender uses a separate WebSocket to
report our WGS84 position and `rel_alt` at 1 Hz.

## Identity and configuration

The 2027 AEAC CONOPS v1.0, page 20, defines `uavId` as a bidder-chosen string.
It requires telemetry whenever armed at 1 Hz with Unix time, decimal-degree
lat/lon, AGL metres, horizontal/vertical accuracy, battery 0–100, mode and
0–1 link statuses. It says server traffic is supplied at 1 Hz with horizontal
and vertical exclusion-cylinder keep-away. Page 22 defines rate penalties
outside 0.4–1.1 seconds. The PDF does **not** define a server `aircraftIndex`
mapping or exact WebSocket JSON; verify both against the competition server's
documentation and real replies. The URL currently in code is inherited from
earlier AEAC work and must also be confirmed for the 2027 server.

`airside/.env` is Git-ignored and intentionally has blank token/ID fields.
Enter the full team `AEAC_CONNECTION_TOKEN`, a chosen `AEAC_UAV_ID`, and the
verified numeric `AEAC_OWN_AIRCRAFT_INDEX` only on the flight computer. Never
commit, paste or bag the token. Index `-1`, blank token or blank ID is no-go.
The token authenticates *both* WebSocket connections; the UAV ID is not a
secret token and is not interchangeable with `aircraftIndex`.

## Run and evidence

For a local mock-server + ArduCopter SITL qualification (no real AEAC token):

```bash
ARTIFACT_DIR=/var/snap/docker/common/pr181-aeac-bendy-sitl \
DOCKER_BIN=/snap/bin/docker AIRSIDE_IMAGE=warg/airside:pr181-bendy \
bash airside/experiments/obstacle_avoidance/live_backup/run_aeac_bendy_sitl.sh traffic_static
```

The runner has `traffic_clear`, `traffic_static`, `traffic_dropout`, and
`traffic_pilot_takeover` scenarios. It requires two simultaneous mock WebSocket
connections, at least ten armed telemetry packets, all armed packet intervals
within 0.4–1.1 s, one MAVROS setpoint owner, a recorded rosbag, and observed
lapping speed at or below 1 m/s. It commands 0.8 m/s to leave headroom below
the 1 m/s first-flight limit. The mock server is a protocol fixture, **not**
proof the 2027 competition server accepts the JSON or authenticates the token.

## Mock-server SITL result (October 7, 2026)

The rebuilt `warg/airside:pr181-bendy` image passed all four fresh-container
scenarios: clear, stationary traffic, 5.5 s traffic dropout, and LOITER pilot
takeover. The sender maintained 0.900–1.001 s armed packet intervals with two
simultaneous WebSockets. Each scenario recorded a rosbag, had one MAVROS
setpoint owner and no keep-away breach. Peak observed horizontal speed was
0.872 m/s or lower, and the stationary/dropout/takeover runs kept at least
1.20 m beyond the traffic keep-away cylinder. Dropout stopped autonomous
motion and resumed only after fresh traffic returned. These are **mock**
results; the real server and physical aircraft remain unverified.

After the final rebuild, `warg/airside:pr181-final` passed the stationary and
5.5 s dropout scenarios again with source-stamped pose speed and a pose-stream
gap gate. Stationary traffic reached the waypoint in 54.333 s, with a
0.827 m/s peak and 1.232 m beyond keep-away; dropout stopped, recovered, and
reached the waypoint in 58.708 s, with a 0.823 m/s peak and 1.223 m beyond
keep-away. Both had two WebSockets and no armed telemetry interval outside
0.4–1.1 s. Artifacts are under
`/var/snap/docker/common/pr181-aeac-bendy-final-static/` and
`/var/snap/docker/common/pr181-aeac-bendy-final-dropout/`.

For props-off integration on the actual flight computer, run from `airside/`:

```bash
docker compose -f compose.yaml -f compose.aeac-bendy.yaml up --build
```

Do not combine it with `compose.flight-test.yaml` or
`compose.aeac-backup.yaml`. This launch starts the ordinary engine and sole
position controller, but no synthetic traffic publisher and no A* controller.
It uses `/aeac/live_traffic` with a 2.5 s freshness limit; startup, malformed,
unhealthy, disconnected or stale traffic must hold motion. An empty *healthy*
snapshot is valid and may permit travel.

## Remaining flight gates

- With props off, verify both real WebSockets, a recent `telemetry_ack`,
  actual 0.4–1.1 s armed-send intervals, real 1 Hz snapshots, correct own index,
  WGS84 coordinates, AGL datum, battery, accuracy and mode. The sender's link
  status values are currently fixed estimates (`1.0`), not measured RF quality;
  do not treat them as a safety claim.
- Confirm the physical aircraft's observed horizontal speed stays below 1 m/s
  with the 0.8 m/s command, and that LOITER/manual takeover stops autonomous
  setpoints. Use a safety pilot and check the site-specific geofence,
  100 m AGL boundary and flight-termination system; this software does not
  implement the CONOPS boundary/termination requirements.
- Review the controlled `traffic_dropout` mock-SITL rosbag and repeat the
  disconnect/stale-snapshot test props-off with the real server before enabling
  live avoidance in a physical flight. The existing stationary obstacle ground
  test is a separate, lower-risk validation step.

No live competition-server or physical-flight approval is claimed by the
mock-server SITL result.
