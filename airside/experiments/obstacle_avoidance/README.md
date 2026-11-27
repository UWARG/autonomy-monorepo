# Moving traffic obstacle avoidance

This directory qualifies the #232-based live AEAC path. It does not replace
the #181 synthetic static-obstacle test. The data flow is:

`AEAC -> comms/traffic_listener -> /aeac/traffic -> position_controller -> GNC`

An independent `comms/telemetry_sender` can report this vehicle's WGS84 GPS
and MAVROS relative altitude at 1 Hz. It is **off by default**; the receive-only
capture and all SITL tests never enable it.

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
it holds and alerts. A failed heading stays excluded until the required
waypoint is 1.5 m closer or the mission advances. If the optional look-ahead
aim is blocked, it replans toward the required waypoint. If a lap waypoint is
inside a live traffic buffer, the controller proposes a point along the route
centroid's outward ray, at most 20 m from the original. The mission manager
accepts it only outside the route polygon and inside the inset flight fence;
the drone must reach it within 3 m before the next leg. The chosen point stays
fixed unless fresh traffic makes it unsafe. Without a safe replacement it
holds and alerts; it never silently skips a leg.

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
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh stall
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh engine-replace
```

Ignored `artifacts/<scenario>-*/` directories contain the summary, logs and
rosbag. A pass requires one setpoint publisher, no raw keep-away entry, at
least 3 m additional measured margin, correct stale hold, and the expected
goal completion or blocked-route alert. The sharp-turn and 15/20 m/s triangle
runs are **stress-only**: their success does not qualify those speeds for
props-on flight, and a hold is not counted as route completion.
`stall` uses a test-only 0.1 m/s FCU setting to verify that an in-flight
retry changes the setpoint heading; it is not a physical-stall qualification.

`listener-moving` and `listener-dropout` start the Airside launch with the
mission manager and camera disabled. A local mock WebSocket supplies 1 Hz AEAC
traffic to the real `traffic_listener`; no real token or server is used. These
tests check the listener-to-controller link, including complete traffic
snapshots and disconnect/reconnect behavior. The direct `moving` test bypasses the
listener.

`engine-triangle` and `engine-eight` additionally start the real mission
manager with one lap and use its waypoint targets and spline path. The runner
uses home `43.4338803,-80.5775947,330,0` for the active WRESTRC triangle;
other routes use their first waypoint. A mock aircraft crosses a full travel
leg at 1 Hz after takeoff. `engine-eight` uses a separate Cornfield fixture. The summary
records target order, path lengths, traffic, clearance,
speed, hold reasons and setpoint publishers. The server feed contains other
aircraft only; no own-aircraft index or filtering is configured.

If rosbag reports multiple types for `/position_controller/target`, inspect
`summary.json`'s `target_publishers` and the failing stack's ROS graph. This
branch uses only `airside_interfaces/msg/PositionTarget` on that topic. Stop
older Airside stacks sharing the ROS domain, rebuild the image, and rerun.
Save the failing commit, launch command and rosbag log if the type still
differs. Only `position_controller` may publish MAVROS position setpoints.
While the failing stack is running, inspect its topic endpoints from `airside/`:

```bash
docker compose -f compose.sitl.yaml exec airside bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   ros2 topic info -v /position_controller/target'
```

Send the output, including each publisher's node name and type, with the
failing commit SHA. Do not send the `.env` or token.

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

To check that the real listener also publishes those snapshots on ROS, run
this receive-only probe. It prints counts and aircraft indices, not names or
the token:

```bash
docker run --rm --network host --ipc host --env-file airside/.env \
  -e ROS_DOMAIN_ID=220 -e ROS_LOCALHOST_ONLY=1 \
  -v "$PWD":/repo:ro -w /repo --entrypoint bash warg/airside:latest -lc \
  'source /opt/ros/humble/setup.bash; source /ros_ws/install/setup.bash; \
   python3 airside/experiments/obstacle_avoidance/probe_live_listener.py --duration-s 15'
```

Use an unused ROS domain ID below 233. This checks reception and ROS
publication, not live SITL route completion.

## Replay a capture

Preselect three start times and use a fresh SITL instance for each run:

```bash
python3 airside/experiments/obstacle_avoidance/server_traffic_replay.py \
  --capture airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  --route airside/src/engine/config/waypoints.yaml \
  --replay-start-s 0

bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh replay \
  airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  --start-s 0
```

The runner starts at the mission start, commands the complete closed route,
preserves all captured aircraft and source timing, and does not loop or invent
data. A blocked run remains a failure to complete the route even when hold and
pilot alert work correctly. The active `waypoints.yaml` and test copy
`routes/wrestrc-triangle.yaml` use the same WRESTRC vertices. The mission
manager orders them clockwise from home. This direct-target replay does not exercise the mission
manager's waypoint-replacement handshake.

For the full mission-manager handshake, use `engine-replay` with the test-only
route. The live listener is disabled so it cannot overwrite the capture:

```bash
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh engine-replay \
  airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  --start-s 0 \
  --route airside/experiments/obstacle_avoidance/routes/wrestrc-triangle.yaml
```

This uses the real controller and mission manager, but sends no telemetry to
the server. A hold or blocked waypoint is not a completed lap.

## Flight gate

Live props-on testing remains disabled until the intended route is confirmed,
all three final-image real-track replays pass on that route, and an authorized
props-off check confirms both WebSockets, WGS84/AGL,
approximately 1 Hz send/receive, and end-to-end
delay. Do not send SITL telemetry to the real server.

For that authorized check only, use a real vehicle with props removed and no
other Airside stack running. In ignored `airside/.env`, set the confirmed
`AEAC_UAV_ID`, approved
`FLIGHT_BOUNDARY_WGS84`, `USE_ENGINE=false`, and
`USE_AEAC_TELEMETRY=true`. From the repository root, run the local check:

```bash
docker run --rm --env-file airside/.env -v "$PWD":/repo:ro -w /repo \
  --entrypoint python3 warg/airside:latest \
  airside/experiments/obstacle_avoidance/check_field_config.py \
  --route airside/src/engine/config/waypoints.yaml --mode props-off
```

This checks settings and geometry only; it does not connect or send. Then,
from `airside/`, run `docker compose up --build`.
Check `/aeac/traffic`, `/aeac/telemetry_diagnostics` and their timestamps;
the telemetry diagnostic must report recent ACKs and roughly 1 s send
intervals. Stop the stack after the check, and restore
`USE_AEAC_TELEMETRY=false` before any SITL run. Never expose the token in logs
or a PR. A missing setting or failed check means no props-on test.

See [RESULTS.md](RESULTS.md) for current evidence and open gates.
