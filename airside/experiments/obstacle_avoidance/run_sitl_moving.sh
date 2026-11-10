#!/usr/bin/env bash
# Kept LF-only via .gitattributes for execution from Windows worktrees.
# Isolated ArduCopter + #232 position-controller SITL test; one scenario per run.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../.." && pwd)"
scenario="${1:?usage: run_sitl_moving.sh clear|moving|multi|turn|dropout|blocked|pilot|replay}"
case "$scenario" in
    clear|moving|multi|turn|dropout|blocked|pilot|replay) ;;
    *) echo "unknown scenario: $scenario" >&2; exit 2 ;;
esac
capture_in_container=""
route_in_container=""
own_index=""
replay_start_s=""
if [[ "$scenario" == replay ]]; then
    [[ -n "${2:-}" && -n "${3:-}" ]] || {
        echo "usage: run_sitl_moving.sh replay <capture.jsonl> <confirmed-own-index> --start-s <seconds> [--route route.yaml]" >&2
        exit 2
    }
    own_index="$3"
    [[ "$own_index" =~ ^[0-9]+$ ]] && (( own_index <= 255 )) || {
        echo "confirmed own aircraft index must be 0..255" >&2
        exit 2
    }
    capture_path="$(realpath "$2")"
    route_path="$repo_root/airside/src/engine/config/waypoints.yaml"
    shift 3
    while (( $# )); do
        case "$1" in
            --start-s)
                [[ $# -ge 2 ]] || { echo "--start-s requires a value" >&2; exit 2; }
                replay_start_s="$2"
                shift 2
                ;;
            --route)
                [[ $# -ge 2 ]] || { echo "--route requires a value" >&2; exit 2; }
                route_path="$2"
                shift 2
                ;;
            *)
                echo "unknown replay option: $1" >&2
                exit 2
                ;;
        esac
    done
    [[ "$replay_start_s" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
        echo "--start-s must be a non-negative number" >&2
        exit 2
    }
    route_path="$(realpath "$route_path")"
    for file_path in "$capture_path" "$route_path"; do
        [[ -f "$file_path" && "$file_path" == "$repo_root/"* ]] || {
            echo "replay input must be an existing file inside this worktree" >&2
            exit 2
        }
    done
    capture_in_container="/repo/${capture_path#"$repo_root/"}"
    route_in_container="/repo/${route_path#"$repo_root/"}"
fi

docker_bin="${DOCKER_BIN:-docker}"
if ! command -v "$docker_bin" >/dev/null 2>&1 && [[ -x /snap/bin/docker ]]; then
    docker_bin=/snap/bin/docker
fi
for attempt in {1..30}; do
    if "$docker_bin" info >/dev/null 2>&1; then
        break
    fi
    sleep 1
done
"$docker_bin" info >/dev/null
artifact_root="${ARTIFACT_DIR:-$repo_root/airside/experiments/obstacle_avoidance/artifacts}"
mkdir -p "$artifact_root"
artifact_dir="$(mktemp -d "$(realpath "$artifact_root")/${scenario}-XXXXXXXX")"
sitl_name="sitl-232-moving-$$"
airside_name="airside-232-moving-$$"
echo "Artifacts: $artifact_dir"

cleanup() {
    "$docker_bin" logs "$sitl_name" > "$artifact_dir/sitl.log" 2>&1 || true
    "$docker_bin" rm -f "$sitl_name" "$airside_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

"$docker_bin" run -d --name "$sitl_name" --network host \
    -v "$script_dir":/test:ro \
    warg/sitl:latest bash -lc \
    'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
     --sim-address=127.0.0.1 \
     --defaults Tools/autotest/default_params/copter.parm,/test/sitl_speed_1p5.parm' \
    > "$artifact_dir/sitl-id.txt"
sleep 3

"$docker_bin" run --rm --name "$airside_name" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-232}" \
    -e ROS_LOCALHOST_ONLY=1 \
    -e SCENARIO="$scenario" \
    -e CAPTURE_FILE="$capture_in_container" \
    -e ROUTE_FILE="$route_in_container" \
    -e OWN_AIRCRAFT_INDEX="$own_index" \
    -e REPLAY_START_S="$replay_start_s" \
    -v "$repo_root":/repo:ro \
    -v "$artifact_dir":/artifacts \
    warg/airside:latest bash -lc '
        source /opt/ros/humble/setup.bash
        source /ros_ws/install/setup.bash
        ros2 run mavros mavros_node --ros-args \
            -p fcu_url:=tcp://127.0.0.1:5760 \
            -p fcu_protocol:=v2.0 \
            -p tgt_system:=1 -p tgt_component:=1 \
            > /artifacts/mavros.log 2>&1 &
        mavros_pid=$!
        ros2 run navigation position_controller \
            > /artifacts/controller.log 2>&1 &
        controller_pid=$!
        setsid ros2 bag record -o /artifacts/rosbag \
            /position_controller/target /position_controller/avoidance_status \
            /aeac/traffic /mavros/state /mavros/local_position/pose \
            /mavros/global_position/global /mavros/global_position/rel_alt \
            /mavros/setpoint_raw/global /mavros/statustext/send \
            > /artifacts/rosbag.log 2>&1 &
        bag_pid=$!
        if [[ "$SCENARIO" == replay ]]; then
            timeout 420 python3 /repo/airside/experiments/obstacle_avoidance/sitl_server_replay.py \
                --capture "$CAPTURE_FILE" --route "$ROUTE_FILE" \
                --own-aircraft-index "$OWN_AIRCRAFT_INDEX" \
                --replay-start-s "$REPLAY_START_S" \
                --summary /artifacts/summary.json
        else
            timeout 420 python3 /repo/airside/experiments/obstacle_avoidance/sitl_moving_readiness.py \
                --scenario "$SCENARIO" --summary /artifacts/summary.json
        fi
        result=$?
        kill -INT -- -$bag_pid >/dev/null 2>&1 || true
        for _ in $(seq 1 40); do
            kill -0 "$bag_pid" >/dev/null 2>&1 || break
            sleep 0.25
        done
        kill -TERM -- -$bag_pid >/dev/null 2>&1 || true
        wait "$bag_pid" >/dev/null 2>&1 || true
        if [[ ! -s /artifacts/rosbag/metadata.yaml ]]; then
            echo "rosbag metadata missing" >&2
            result=1
        fi
        kill -TERM "$controller_pid" "$mavros_pid" >/dev/null 2>&1 || true
        exit "$result"
    ' > "$artifact_dir/runner.log" 2>&1 || {
        cat "$artifact_dir/runner.log"
        exit 1
    }

cat "$artifact_dir/summary.json"
