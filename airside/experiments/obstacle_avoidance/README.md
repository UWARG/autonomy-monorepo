# Moving traffic obstacle avoidance

This directory qualifies the #232-based live AEAC path. It does not replace
the #181 synthetic static-obstacle test. The data flow is:

`AEAC -> comms/traffic_listener -> /aeac/traffic -> position_controller -> GNC`

The controller is the only MAVROS position-setpoint publisher. Live traffic
keeps the server radius and adds 3 m for stationary traffic or 5.75 m for
moving traffic. Moving positions are age-compensated and swept for 3 s. A
snapshot older than 1.5 s causes a hold. No path causes hold, replan on each
new snapshot, and a pilot alert. Holding is not called safe when traffic is
predicted to enter the hold point.

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

Live props-on testing remains disabled until all three final-image real-track
replays pass and an authorized props-off check confirms both WebSockets, the
own-aircraft index, WGS84/AGL, approximately 1 Hz send/receive, and end-to-end
delay. Do not send SITL telemetry to the real server.

See [RESULTS.md](RESULTS.md) for current evidence and open gates.
