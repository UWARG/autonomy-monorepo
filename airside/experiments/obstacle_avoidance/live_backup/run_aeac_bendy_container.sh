#!/usr/bin/env bash
set -euo pipefail

scenario="${SCENARIO:?SCENARIO is required}"
case "$scenario" in
    traffic_clear) mock_mode=clear ;;
    traffic_static|traffic_pilot_takeover) mock_mode=static ;;
    traffic_dropout) mock_mode=dropout ;;
    *) exit 2 ;;
esac
script_dir=/repo/airside/experiments/obstacle_avoidance/live_backup
export OBSTACLE_TRAFFIC_TOPIC=/aeac/live_traffic
export OBSTACLE_HORIZONTAL_SPEED_MPS=0.8
export OBSTACLE_SPEED_GATE_MPS=1.0
export AEAC_CONNECTION_TOKEN=sitl-only-token
export AEAC_UAV_ID=WARG-SITL
export AEAC_OWN_AIRCRAFT_INDEX=1
export AEAC_WEBSOCKET_URL=ws://127.0.0.1:8765

python3 "$script_dir/mock_aeac_server.py" \
    --mode "$mock_mode" --token sitl-only-token \
    --summary "/artifacts/$scenario-server.json" \
    > "/artifacts/$scenario-server.log" 2>&1 &
server_pid=$!
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=tcp://127.0.0.1:5760 \
    -p fcu_protocol:=v2.0 \
    -p tgt_system:=1 -p tgt_component:=1 \
    > "/artifacts/$scenario-mavros.log" 2>&1 &
mavros_pid=$!
ros2 run comms traffic_listener \
    > "/artifacts/$scenario-listener.log" 2>&1 &
listener_pid=$!
ros2 run comms telemetry_sender \
    > "/artifacts/$scenario-sender.log" 2>&1 &
sender_pid=$!
setsid ros2 bag record -o "/artifacts/$scenario-rosbag" \
    /aeac/live_traffic \
    /position_controller/obstacle \
    /position_controller/obstacle_snapshot \
    /aeac/telemetry_diagnostics \
    /obstacle_avoidance/diagnostics \
    /mavros/state \
    /mavros/global_position/global \
    /mavros/global_position/rel_alt \
    /mavros/local_position/pose \
    /position_controller/velocity_target \
    /mavros/setpoint_velocity/cmd_vel \
    /mavros/setpoint_raw/global \
    > "/artifacts/$scenario-rosbag.log" 2>&1 &
bag_pid=$!

status=0
python3 /repo/airside/experiments/obstacle_avoidance/airside_bt_sitl.py \
    --scenario "$scenario" --duration 90 --readiness-timeout 180 \
    --log-jsonl "/artifacts/$scenario.jsonl" \
    --summary-json "/artifacts/$scenario-summary.json" \
    --params-json "/artifacts/$scenario-params.json" \
    --waypoints-file "/artifacts/$scenario-waypoints.yaml" || status=$?

kill -INT -- -"$bag_pid" >/dev/null 2>&1 || true
for attempt in {1..40}; do
    kill -0 "$bag_pid" >/dev/null 2>&1 || break
    sleep 0.25
done
kill -TERM -- -"$bag_pid" >/dev/null 2>&1 || true
wait "$bag_pid" >/dev/null 2>&1 || true
if [[ ! -s "/artifacts/$scenario-rosbag/metadata.yaml" ]]; then
    echo "ROS bag metadata missing" >&2
    status=1
fi
if [[ ! -s "/artifacts/$scenario-server.json" ]]; then
    echo "Mock server received no telemetry" >&2
    status=1
else
    python3 -c 'import json,sys; s=json.load(open(sys.argv[1])); assert s["connections"]>=2 and s["armed_telemetry_count"]>=10 and s["armed_intervals_outside_0_4_to_1_1"]==0, s; print("AEAC mock summary:", {k:v for k,v in s.items() if k!="last_payload"})' \
        "/artifacts/$scenario-server.json" || status=1
fi
kill -TERM "$sender_pid" "$listener_pid" "$mavros_pid" "$server_pid" \
    >/dev/null 2>&1 || true
wait "$sender_pid" "$listener_pid" "$mavros_pid" "$server_pid" \
    >/dev/null 2>&1 || true
exit "$status"
