# #232-based moving traffic flight-test preparation

This is a personal-branch qualification path. It does not change #181's
previously validated imaginary static-obstacle path or #203. Here, the
`comms/traffic_listener` publishes complete `/aeac/traffic` snapshots, the
`navigation/position_controller` publishes the only MAVROS position setpoint,
and `gnc/obstacle_avoidance` expands the server keep-away for 1 Hz live traffic.

The [2027 AEAC CONOPS](https://www.aerialevolution.ca/wp-content/uploads/2026/09/2027-AEAC-CONOPS-v1.0-2026-09-08.pdf)
requires 1 Hz telemetry while armed, describes 1 Hz simulated traffic, and
forbids entering each simulated aircraft's keep-away cylinder. The code's
1.5 s traffic timeout, 5 m minimum extra buffer, 3 s prediction and 1 m
planner clearance are additional conservative test settings, **not** a
guarantee of collision avoidance. Holding position can itself be unsafe when
traffic approaches; the controller asks for pilot takeover in that case.
For cross-traffic, a separate 20 s closest-approach gate yields before entering
the crossing path when the hold point is predicted safe; it does not replace
the 3 s swept-corridor planner prediction.

## Build and local tests

From the repository root, build the Airside image from **this** worktree:

```bash
cd airside
docker compose -f compose.sitl.yaml build
cd ..
uv run --project gnc --extra dev pytest gnc/tests
docker run --rm warg/airside:latest bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   cd /ros_ws && colcon test && colcon test-result --verbose'
```

## Read-only real-server capture

The capture tool opens one authenticated WebSocket and **never sends a frame**.
It saves only traffic indices, WGS84/AGL, motion, keep-away, receipt times and
parse problems; it omits names and the token. Review the current server docs at
`aeac.mylonics.com` and the endpoint before connecting. Put the real token in
`airside/.env`, which is ignored by Git; never paste it into a command line,
commit, rosbag, or PR. `AEAC_OWN_AIRCRAFT_INDEX` must be confirmed before
autonomous live avoidance, even though read-only capture can run without it.

```bash
docker run --rm --network host --env-file airside/.env \
  -v "$PWD":/repo -w /repo warg/airside:latest \
  python3 airside/experiments/obstacle_avoidance/capture_server_traffic.py \
  --duration-s 180 \
  --output airside/experiments/obstacle_avoidance/captures/receive-only.jsonl
```

Exit code 2 means no moving aircraft was observed; it is **not** a passed
server test. Do not start `aeac_telemetry` or send SITL telemetry to the real
server during this read-only test. The official server may provide no useful
traffic until a team flight is active; if so, record the gap rather than
claiming that mock traffic came from AEAC.

## Match a captured track to the route and replay it

The matcher uses the Airside lap waypoints, including the closing leg, to
identify a moving aircraft index whose keep-away plus live buffer intersects
the route. It reports the matching leg, capture interval range, and whether
there is enough lead-in and duration to replay a complete leg. Use a
confirmed `AEAC_OWN_AIRCRAFT_INDEX`; the tool does not infer our index.
If the server test uses a different course, pass that course's waypoint
YAML to both commands.

```bash
python3 airside/experiments/obstacle_avoidance/server_traffic_replay.py \
  --capture airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  --route airside/src/engine/config/waypoints.yaml \
  --own-aircraft-index <confirmed-index>

bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh replay \
  airside/experiments/obstacle_avoidance/captures/receive-only.jsonl \
  <confirmed-index>
```

Run the replay three times with a fresh SITL boot each time. The runner maps
the matched route leg and captured aircraft into the same local SITL frame
and publishes snapshots at their original relative times. Continuous
simulator truth is interpolated between captured positions for distance
measurement. This is a replay model, not ground truth of the original AEAC
simulation between 1 Hz updates. The JSON summary records traffic cadence,
the 1.5 m/s FCU readback, observed speed, raw keep-away margin, hold reasons,
and pilot alerts. A persistently blocked leg fails goal completion; inspect
its hold and takeover evidence rather than treating hover as a successful
avoidance run.

## Mock moving traffic in ArduCopter SITL

The pinned `warg/sitl:latest` image uses legacy `WPNAV_SPEED` in cm/s.
`sitl_speed_1p5.parm` sets 150 cm/s, and the harness reads the FCU value back.
The position-only setpoint does not itself impose a speed cap. If using a
different ArduPilot version, select and read back its supported `WP_SPD` or
`WPNAV_SPEED` parameter before any test; never assume the units.

```bash
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh clear
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh moving
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh moving
bash airside/experiments/obstacle_avoidance/run_sitl_moving.sh moving
```

Also run `multi`, `turn`, `dropout`, `blocked`, and `pilot`. Every run uses new
SITL and Airside containers and writes ignored `artifacts/<scenario>-*/`
with a rosbag, logs and a JSON summary. Passing criteria: raw keep-away never
entered, at least 3 m additional measured margin, speed readback 1.5 m/s,
observed peak at most 1.7 m/s, only one setpoint publisher, reachable goal
completed, blocked route held with pilot alert, stale traffic held, and pilot
takeover stopped setpoints. `clear` validates the harness before traffic tests.

Captured real-server traffic is **not** yet a SITL pass merely because the
capture tool received it. Replay must preserve genuine time/trajectory data,
put it into the SITL geographic frame, and pass the same measured-distance
checks before making that claim.

The dated qualification evidence and remaining hard gates are recorded in
[`RESULTS.md`](RESULTS.md). Generated bags and per-run summaries remain local
under the ignored `artifacts/` directory.

## Flight-test gate

No props-on live test until the real-server moving-traffic capture/replay,
mock moving scenarios, static #181 regression, and props-off two-WebSocket
check all pass. The props-off check also verifies own aircraft index, WGS84
latitude/longitude, AGL convention, approximately 1 Hz receive/send, and
end-to-end delay. Sending to the real server requires separate confirmation
of its official test procedure. The first physical speed gate remains 1 m/s;
1.5 m/s is a SITL trial only.
