#!/usr/bin/env bash
set -euo pipefail

repo_root="${1:-/repo}"
artifact_dir="${2:-/tmp/aeac-bridge-smoke}"
mkdir -p "$artifact_dir"

server_pid=""
bridge_pid=""
publisher_pids=()
cleanup() {
    for pid in "${publisher_pids[@]}"; do
        kill -INT "$pid" >/dev/null 2>&1 || true
    done
    [[ -z "$bridge_pid" ]] || kill -INT "$bridge_pid" >/dev/null 2>&1 || true
    [[ -z "$server_pid" ]] || kill -INT "$server_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

python3 "$repo_root/airside/scripts/avoidance/fake_aeac_server.py" \
    --scenario static \
    --port 8765 \
    --transcript "$artifact_dir/transcript.jsonl" \
    >"$artifact_dir/server.log" 2>&1 &
server_pid=$!

ros2 run aeac_bridge bridge --ros-args \
    -p aeac_websocket_url:=ws://127.0.0.1:8765/test \
    -p aeac_connection_token:=local-test-token \
    -p uav_id:=WARG-01 \
    -p own_aircraft_index:=1 \
    -p protocol_verified:=true \
    >"$artifact_dir/bridge.log" 2>&1 &
bridge_pid=$!

ros2 topic pub -r 5 /mavros/state mavros_msgs/msg/State \
    '{connected: true, armed: true, mode: GUIDED}' \
    >"$artifact_dir/state-publisher.log" 2>&1 &
publisher_pids+=("$!")
ros2 topic pub -r 5 /mavros/global_position/global sensor_msgs/msg/NavSatFix \
    '{status: {status: 0}, latitude: 43.0, longitude: -80.0, altitude: 315.0}' \
    >"$artifact_dir/fix-publisher.log" 2>&1 &
publisher_pids+=("$!")
ros2 topic pub -r 5 /mavros/global_position/rel_alt std_msgs/msg/Float64 \
    '{data: 15.0}' \
    >"$artifact_dir/altitude-publisher.log" 2>&1 &
publisher_pids+=("$!")

sleep 3
python3 "$repo_root/airside/scripts/avoidance/traffic_snapshot_probe.py" \
    --timeout 10 \
    --output "$artifact_dir/traffic.json" \
    >"$artifact_dir/probe.log" 2>&1

grep -q '"healthy": true' "$artifact_dir/traffic.json"
grep -q '"own_aircraft_index": 1' "$artifact_dir/traffic.json"
grep -q '"aircraft_indices"' "$artifact_dir/traffic.json"
grep -q '"action": "telemetry"' "$artifact_dir/transcript.jsonl"

cat "$artifact_dir/traffic.json"
