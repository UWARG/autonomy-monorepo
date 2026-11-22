# Moving traffic obstacle avoidance

This directory qualifies the #232-based live AEAC path. It does not replace
the #181 synthetic static-obstacle test. The data flow is:

`AEAC -> comms/traffic_listener -> /aeac/traffic -> position_controller -> GNC`

The controller is the only MAVROS position-setpoint publisher. Live traffic
keeps the server radius. A moving aircraft has both a 3 s straight-line sweep
and a heading-independent reachability disc around its last observed position:
the disc adds at least 5.75 m, or speed times (snapshot age + 4.5 s), whichever
is larger. The 4.5 s response bound must cover the allowed 1.5 s update gap
plus measured FCU stopping time before flight. A snapshot older than 1.5 s,
impossible position jump, or traffic over the current 5 m/s test envelope
causes a hold and pilot alert.
The 15/20 m/s runs are unsupported stress cases, not autonomous flight cases.
If the hold point may be entered, the controller immediately asks the pilot
to take over; hover is not claimed to be safe avoidance.

The controller also requires `FLIGHT_BOUNDARY_WGS84` in ignored `airside/.env`:
an approved horizontal polygon as `[[lat,lon],[lat,lon],...]`. It holds and
alerts if the polygon is missing/invalid, the vehicle or mission target is
outside its 5 m inward margin, or the next setpoint segment crosses that
margin. This companion guard is **not** a hard FCU geofence; configure and
verify the flight controller's fence separately before physical flight. The
runner creates a wide SITL-only polygon, not a field-approved boundary.

On one leg, spline aim distance is monotonic so GPS noise cannot pull it
backward. After 8 s of commanded path following without 1.5 m net progress,
the controller excludes the last heading and tries another clearance-checked
candidate. It never shrinks the traffic buffer. If there is no safe alternate,
it holds and alerts. If only the optional look-ahead aim is blocked, it
replans toward the current required waypoint. If that waypoint is inside the
traffic buffer, it holds, replans on new snapshots and alerts; it never skips
the waypoint.

The planner uses 2 m/s for conservative relative-motion calculations. SITL
sets the FCU to 1.5 m/s and accepts a measured peak up to 1.7 m/s. The first
physical flight remains limited to 1 m/s.

## Build and test

Run from the repository root:

```bash
cd airside
docker compose -f compose.sitl.yaml build
cd ..
uv run --project gnc --extra dev pytest gnc/tests
docker run --rm -v "$PWD":/repo:ro warg/airside:latest \
  bash /repo/airside/experiments/obstacle_avoidance/run_colcon_test.sh
```

Run each mock scenario in a fresh SITL instance:

```bash
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh clear
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh moving  # run 3 times
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh listener-moving  # run 3 times
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh listener-dropout
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh engine-triangle
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh engine-eight
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh multi
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh turn
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh sharp_turn
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh triangle15
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh triangle20
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh listener-triangle15
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh dropout
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh blocked
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh pilot
```

Ignored `artifacts/<scenario>-*/` directories contain the summary, logs and
rosbag. A pass requires one setpoint publisher, no raw keep-away entry, at
least 3 m additional measured margin, correct stale hold, and the expected
goal completion or blocked-route alert. The sharp-turn and 15/20 m/s triangle
runs are **stress-only**: their success does not qualify those speeds for
props-on flight, and a hold is not counted as route completion.

`listener-moving` and `listener-dropout` start the Airside launch with the
mission manager and camera disabled. A local mock WebSocket supplies 1 Hz AEAC
traffic to the real `traffic_listener`; no real token or server is used. These
tests check the listener-to-controller link, including own-aircraft filtering
and disconnect/reconnect behavior. The direct `moving` test bypasses the
listener.

`engine-triangle` and `engine-eight` additionally start the real mission
manager with one lap and use its waypoint targets and spline path. The runner
places SITL at the route's first waypoint and generates a moving 1 Hz crossing
on its first travel leg. After syncing #232, the active route is the eight-point
Cornfield route; `engine-eight` uses an equivalent test fixture. The summary
records target order, path lengths, traffic, clearance,
speed, hold reasons and setpoint publishers. These local tests still do not
reproduce an unidentified merged copy or qualify real server traffic.

If a merged copy fails, save its commit SHA, launch command, controller log,
and rosbag. Check that exactly one node publishes
`/mavros/setpoint_raw/global` and that the launched listener publishes the
topic the controller subscribes to. This branch uses `/aeac/traffic`; another
implementation may use `/position_controller/obstacle`. A passing test on
this branch does not identify the cause of a different copy's failure.

## Receive-only AEAC capture

Put the token in ignored `airside/.env`; never put it in a command, log,
rosbag, commit, or PR. Confirm the endpoint before connecting. This command
receives traffic and sends no telemetry:

```bash
docker run --rm --network host --env-file airside/.env \
  -v "$PWD":/repo -w /repo warg/airside:latest \
  python3 airside/experiments/obstacle_avoidance/capture_server_traffic.py \
  --duration-s 720 \
  --output airside/experiments/obstacle_avoidance/captures/receive-only.jsonl
```

Exit code 2 means that no moving traffic was observed. Captures contain
traffic fields and timing but omit names and the token.

## Replay a capture

Use the confirmed own-aircraft index. Never infer it from a name. Preselect
three start times and use a fresh SITL instance for each run:

```bash
python3 airside/experiments/obstacle_avoidance/server_traffic_replay.py \
  --capture airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  --route airside/src/engine/config/waypoints.yaml \
  --own-aircraft-index <confirmed-index> --replay-start-s 0

bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh replay \
  airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  <confirmed-index> --start-s 0
```

The runner starts at the mission start, commands the complete closed route,
preserves all captured aircraft and source timing, and does not loop or invent
data. A blocked run remains a failure to complete the route even when hold and
pilot alert work correctly.

## Flight gate

Live props-on testing remains disabled until the intended route is confirmed,
all three final-image real-track replays pass on that route, and an authorized
props-off check confirms both WebSockets, the own-aircraft index, WGS84/AGL,
approximately 1 Hz send/receive, and end-to-end
delay. Do not send SITL telemetry to the real server.

See [RESULTS.md](RESULTS.md) for current evidence and open gates.
