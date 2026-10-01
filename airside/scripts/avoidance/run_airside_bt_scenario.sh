#!/usr/bin/env bash
# Run one airside behavior-tree avoidance scenario with fresh containers.
set -uo pipefail

cd "$(dirname "$0")"

scenario="${1:?scenario is required}"
label="${2:-$scenario}"
scenario_duration="${DEMO_DURATION_S:-90}"
if [[ "$scenario" == "transition" && -z "${DEMO_DURATION_S:-}" ]]; then
    scenario_duration=150
fi
artifact_dir="${ARTIFACT_DIR:?ARTIFACT_DIR is required}"
repo_root="$(cd ../../.. && pwd)"
docker_bin="${DOCKER_BIN:-docker}"
sitl_name="sitl-144-airside"
airside_name="airside-144-test"
airside_image="${AIRSIDE_IMAGE:-warg/airside:latest}"
traffic_scenario=""
own_aircraft_index=1
if [[ "$scenario" == "traffic_unknown_identity" ]]; then
    traffic_scenario="static"
    own_aircraft_index=-1
elif [[ "$scenario" == "traffic_pilot_takeover" ]]; then
    traffic_scenario="static"
elif [[ "$scenario" == traffic_* ]]; then
    traffic_scenario="${scenario#traffic_}"
fi

mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"

cleanup() {
    "$docker_bin" rm -f "$airside_name" >/dev/null 2>&1 || true
    "$docker_bin" rm -f "$sitl_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

cleanup

"$docker_bin" run -d --name "$sitl_name" --network host \
    -v "$PWD":/demo:ro \
    warg/sitl:latest bash -lc \
    'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
     --sim-address=127.0.0.1 \
     --defaults Tools/autotest/default_params/copter.parm,/demo/sitl_airside.parm' \
    >"$artifact_dir/${label}-sitl-container-id.txt"
"$docker_bin" inspect "$sitl_name" \
    >"$artifact_dir/${label}-sitl-inspect.json" 2>&1 || true
sleep 3

"$docker_bin" run --name "$airside_name" --network host --ipc host \
    --entrypoint /bin/bash \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}" \
    -e ROS_LOCALHOST_ONLY=1 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$repo_root":/repo:ro \
    -v "$artifact_dir":/artifacts \
    "$airside_image" -lc \
    "source /opt/ros/humble/setup.bash
     source /ros_ws/install/setup.bash
     export PYTHONPATH=/monorepo\${PYTHONPATH:+:\$PYTHONPATH}
     fake_server_pid=''
     bridge_pid=''
     if [[ -n '${traffic_scenario}' ]]; then
       setsid python3 /repo/airside/scripts/avoidance/fake_aeac_server.py \
         --scenario '${traffic_scenario}' \
         --port 8765 \
         --transcript /artifacts/${label}-aeac-transcript.jsonl \
         > /artifacts/${label}-aeac-server.log 2>&1 &
       fake_server_pid=\$!
       setsid ros2 run aeac_bridge bridge --ros-args \
         -p aeac_websocket_url:=ws://127.0.0.1:8765/test \
         -p aeac_connection_token:=local-test-token \
         -p uav_id:=WARG-01 \
         -p own_aircraft_index:=${own_aircraft_index} \
         -p protocol_verified:=true \
         > /artifacts/${label}-aeac-bridge.log 2>&1 &
       bridge_pid=\$!
     fi
     ros2 run mavros mavros_node --ros-args \
       -p fcu_url:=tcp://127.0.0.1:5760 \
       -p fcu_protocol:=v2.0 \
       -p tgt_system:=1 \
       -p tgt_component:=1 \
       -p plugin_denylist:=['rc_io'] \
       > /artifacts/${label}-mavros.log 2>&1 &
     mavros_pid=\$!
     setsid ros2 bag record \
       --output /artifacts/${label}-rosbag \
       /obstacle_avoidance/scan \
       /aeac/traffic \
       /obstacle_avoidance/diagnostics \
       /mavros/state \
       /mavros/local_position/pose \
       /mavros/global_position/global \
       /mavros/global_position/rel_alt \
       /position_controller/velocity_target \
       /mavros/setpoint_velocity/cmd_vel \
       /mavros/setpoint_raw/global \
       > /artifacts/${label}-rosbag.log 2>&1 &
     bag_pid=\$!
     python3 /repo/airside/scripts/avoidance/airside_bt_sitl.py \
       --scenario ${scenario} \
       --duration ${scenario_duration} \
       --readiness-timeout ${READINESS_TIMEOUT_S:-180} \
       --log-jsonl /artifacts/${label}.jsonl \
       --summary-json /artifacts/${label}-summary.json \
       --params-json /artifacts/${label}-params.json \
       --waypoints-file /artifacts/${label}-waypoints.yaml
     scenario_status=\$?
     if ! kill -0 \$bag_pid >/dev/null 2>&1; then
       echo 'ROS bag recorder exited before scenario completion' >&2
       scenario_status=1
     fi
     kill -INT -- -\$bag_pid >/dev/null 2>&1 || true
     for _ in \$(seq 1 40); do
       kill -0 \$bag_pid >/dev/null 2>&1 || break
       sleep 0.25
     done
     kill -TERM -- -\$bag_pid >/dev/null 2>&1 || true
     wait \$bag_pid >/dev/null 2>&1 || true
     if [[ ! -s /artifacts/${label}-rosbag/metadata.yaml ]]; then
       echo 'ROS bag metadata was not produced' >&2
       scenario_status=1
     fi
     kill -INT \$mavros_pid >/dev/null 2>&1 || true
     for _ in \$(seq 1 20); do
       kill -0 \$mavros_pid >/dev/null 2>&1 || break
       sleep 0.25
     done
     kill -TERM \$mavros_pid >/dev/null 2>&1 || true
     sleep 1
     kill -KILL \$mavros_pid >/dev/null 2>&1 || true
     wait \$mavros_pid >/dev/null 2>&1 || true
     if [[ -n \$bridge_pid ]]; then
       kill -INT -- -\$bridge_pid >/dev/null 2>&1 || true
       for _ in \$(seq 1 20); do
         kill -0 \$bridge_pid >/dev/null 2>&1 || break
         sleep 0.25
       done
       kill -TERM -- -\$bridge_pid >/dev/null 2>&1 || true
       wait \$bridge_pid >/dev/null 2>&1 || true
     fi
     if [[ -n \$fake_server_pid ]]; then
       kill -INT -- -\$fake_server_pid >/dev/null 2>&1 || true
       for _ in \$(seq 1 20); do
         kill -0 \$fake_server_pid >/dev/null 2>&1 || break
         sleep 0.25
       done
       kill -TERM -- -\$fake_server_pid >/dev/null 2>&1 || true
       wait \$fake_server_pid >/dev/null 2>&1 || true
     fi
     exit \$scenario_status" \
    >"$artifact_dir/${label}-runner.log" 2>&1
scenario_status=$?

echo "$scenario_status" >"$artifact_dir/${label}-exit.txt"
"$docker_bin" logs "$sitl_name" \
    >"$artifact_dir/${label}-fc.log" 2>&1 || true
"$docker_bin" inspect "$airside_name" \
    >"$artifact_dir/${label}-airside-inspect.json" 2>&1 || true
cat "$artifact_dir/${label}-runner.log"
exit "$scenario_status"
