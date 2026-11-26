# Moving traffic obstacle avoidance

This directory qualifies the #232-based live AEAC path. It does not replace
the #181 synthetic static-obstacle test. The data flow is:

`AEAC -> comms/traffic_listener -> /aeac/traffic -> position_controller -> GNC`

The controller is the only MAVROS position-setpoint publisher. Live traffic
keeps the server radius and adds 5.75 m for moving traffic; stationary traffic
gets no extra buffer. Moving positions are age-compensated and swept for 3 s.
Each candidate leg also gets a constant-velocity closest-approach check over
its estimated travel time, capped at 20 s. Conflicting candidates are
discarded; if dynamic conflicts eliminate all candidates, the controller
yields only when the hold point is predicted safe. Geometric blockage still
causes a no-path hold. A snapshot older than 1.5 s causes a hold.

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
docker run --rm warg/airside:latest bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   cd /ros_ws && colcon test && colcon test-result --verbose'
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
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh dropout
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh blocked
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh pilot
```

Ignored `artifacts/<scenario>-*/` directories contain the summary, logs and
rosbag. A pass requires one setpoint publisher, no raw keep-away entry, at
least 3 m additional measured margin, correct stale hold, and the expected
goal completion or blocked-route alert.

`listener-moving` and `listener-dropout` start the Airside launch with the
mission manager and camera disabled. A local mock WebSocket supplies 1 Hz AEAC
traffic to the real `traffic_listener`; no real token or server is used. These
tests check the listener-to-controller link, including own-aircraft filtering
and disconnect/reconnect behavior. The direct `moving` test bypasses the
listener.

`engine-triangle` and `engine-eight` additionally start the real mission
manager with one lap and use its waypoint targets and spline path. The runner
places SITL at the route's first waypoint and generates a moving 1 Hz crossing
on its first travel leg. `engine-eight` uses a test fixture with the eight
waypoints from #232; it does not change this branch's active three-point
route. The summary records target order, path lengths, traffic, clearance,
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
