#!/usr/bin/env bash
# Fresh ArduCopter and Airside containers for each PR #181 live-backup scenario.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../../.." && pwd)"
artifact_root="${ARTIFACT_DIR:-$repo_root/airside/log/aeac-backup-sitl}"
docker_bin="${DOCKER_BIN:-docker}"
airside_image="${AIRSIDE_IMAGE:-warg/airside:pr181-live}"
for attempt in {1..12}; do
    if "$docker_bin" info >/dev/null 2>&1; then
        break
    fi
    sleep 3
done
"$docker_bin" info >/dev/null
mkdir -p "$artifact_root"
artifact_dir="$(mktemp -d "$(realpath "$artifact_root")/run-XXXXXXXX")"
echo "Artifacts: $artifact_dir"

if [[ $# -eq 0 || "$1" == "all" ]]; then
    scenarios=(clear detour dropout no_path pilot_takeover)
else
    scenarios=("$1")
fi

for scenario in "${scenarios[@]}"; do
    case "$scenario" in
        clear|detour|dropout|no_path|pilot_takeover) ;;
        *) echo "Unknown scenario: $scenario" >&2; exit 2 ;;
    esac
    sitl_name="sitl-181-live-$scenario"
    airside_name="airside-181-live-$scenario"
    "$docker_bin" rm -f "$sitl_name" "$airside_name" >/dev/null 2>&1 || true
    cleanup() {
        "$docker_bin" logs "$sitl_name" > "$artifact_dir/$scenario-fcu.log" 2>&1 || true
        "$docker_bin" rm -f "$sitl_name" "$airside_name" >/dev/null 2>&1 || true
    }
    trap cleanup EXIT INT TERM

    "$docker_bin" run -d --name "$sitl_name" --network host \
        -v "$script_dir":/test:ro \
        warg/sitl:latest bash -lc \
        'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
         --sim-address=127.0.0.1 \
         --defaults Tools/autotest/default_params/copter.parm,/test/sitl_airside.parm' \
        > "$artifact_dir/$scenario-sitl-id.txt"
    sleep 3

    result=0
    "$docker_bin" run --rm --name "$airside_name" --network host --ipc host \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-203}" \
        -e ROS_LOCALHOST_ONLY=1 \
        -e SCENARIO="$scenario" \
        -v "$repo_root":/repo:ro \
        -v "$artifact_dir":/artifacts \
        "$airside_image" bash -lc '
            ros2 run mavros mavros_node --ros-args \
                -p fcu_url:=tcp://127.0.0.1:5760 \
                -p fcu_protocol:=v2.0 \
                -p tgt_system:=1 \
                -p tgt_component:=1 \
                > "/artifacts/$SCENARIO-mavros.log" 2>&1 &
            mavros_pid=$!
            ros2 run navigation aeac_backup_controller --ros-args \
                -p traffic_required:=true \
                -p own_aircraft_index:=1 \
                > "/artifacts/$SCENARIO-controller.log" 2>&1 &
            controller_pid=$!
            setsid ros2 bag record \
                -o "/artifacts/$SCENARIO-rosbag" \
                /position_controller/target \
                /position_controller/obstacle \
                /position_controller/obstacle_snapshot \
                /position_controller/diagnostics \
                /mavros/state \
                /mavros/local_position/pose \
                /mavros/global_position/global \
                /mavros/global_position/rel_alt \
                /mavros/setpoint_raw/global \
                > "/artifacts/$SCENARIO-rosbag.log" 2>&1 &
            bag_pid=$!
            timeout 420 python3 /repo/airside/experiments/obstacle_avoidance/live_backup/sitl_readiness.py \
                --scenario "$SCENARIO" \
                --summary "/artifacts/$SCENARIO-summary.json"
            scenario_status=$?
            kill -INT -- -$bag_pid >/dev/null 2>&1 || true
            for _ in $(seq 1 40); do
                kill -0 "$bag_pid" >/dev/null 2>&1 || break
                sleep 0.25
            done
            kill -TERM -- -$bag_pid >/dev/null 2>&1 || true
            wait "$bag_pid" >/dev/null 2>&1 || true
            if [[ ! -s "/artifacts/$SCENARIO-rosbag/metadata.yaml" ]]; then
                echo "ROS bag metadata missing" >&2
                scenario_status=1
            fi
            kill -TERM "$controller_pid" "$mavros_pid" >/dev/null 2>&1 || true
            exit "$scenario_status"
        ' > "$artifact_dir/$scenario-runner.log" 2>&1 || result=$?

    cleanup
    trap - EXIT INT TERM
    echo "$result" > "$artifact_dir/$scenario-exit.txt"
    cat "$artifact_dir/$scenario-runner.log"
    if [[ "$result" -ne 0 ]]; then
        echo "Scenario $scenario failed; see $artifact_dir" >&2
        exit "$result"
    fi
done
