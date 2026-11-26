#!/usr/bin/env bash
# Kept LF-only via .gitattributes for execution from Windows worktrees.
# Isolated ArduCopter + #232 position-controller SITL test; one scenario per run.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../.." && pwd)"
scenario="${1:?usage: run_sitl_moving.sh clear|moving|listener-moving|listener-dropout|listener-triangle15|listener-triangle20|engine-triangle|engine-eight|engine-replace|engine-replay|multi|turn|sharp_turn|triangle15|triangle20|dropout|blocked|pilot|stall|replay}"
case "$scenario" in
    clear|moving|listener-moving|listener-dropout|listener-triangle15|listener-triangle20|engine-triangle|engine-eight|engine-replace|engine-replay|multi|turn|sharp_turn|triangle15|triangle20|dropout|blocked|pilot|stall|replay) ;;
    *) echo "unknown scenario: $scenario" >&2; exit 2 ;;
esac
capture_in_container=""
route_in_container=""
own_index=""
replay_start_s=""
sitl_home=""
sitl_param_file=sitl_speed_1p5.parm
if [[ "$scenario" == stall ]]; then sitl_param_file=sitl_stall_speed.parm; fi
if [[ "$scenario" == engine-triangle ]]; then
    route_path="$repo_root/airside/src/engine/config/waypoints.yaml"
elif [[ "$scenario" == engine-eight || "$scenario" == engine-replace ]]; then
    route_path="$script_dir/routes/cornfield-eight.yaml"
fi
if [[ "$scenario" == engine-* && "$scenario" != engine-replay ]]; then
    route_in_container="/repo/${route_path#"$repo_root/"}"
    home_lat="$(awk '/^[[:space:]]*- lat:/ { print $3; exit }' "$route_path")"
    home_lon="$(awk '/^[[:space:]]*lon:/ { print $2; exit }' "$route_path")"
    [[ -n "$home_lat" && -n "$home_lon" ]] || {
        echo "could not read first route coordinate from $route_path" >&2
        exit 2
    }
    sitl_home="$home_lat,$home_lon,300,0"
fi
if [[ "$scenario" == replay || "$scenario" == engine-replay ]]; then
    [[ -n "${2:-}" && -n "${3:-}" ]] || {
        echo "usage: run_sitl_moving.sh replay|engine-replay <capture.jsonl> <diagnostic-own-index> --start-s <seconds> [--route route.yaml]" >&2
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

# SITL-only polygon around the known test origin. It is not an approved
# physical-flight boundary; the live launch must receive a site-approved one.
if [[ -z "$sitl_home" ]]; then
    if [[ "$scenario" == replay || "$scenario" == engine-replay ]]; then
        home_lat="$(awk '/^[[:space:]]*- lat:/ { print $3; exit }' "$route_path")"
        home_lon="$(awk '/^[[:space:]]*lon:/ { print $2; exit }' "$route_path")"
    else
        home_lat=43.477773
        home_lon=-80.5685257
    fi
    sitl_home="$home_lat,$home_lon,300,0"
fi
flight_boundary="$(awk -v lat="$home_lat" -v lon="$home_lon" 'BEGIN {
    printf "[[%.7f,%.7f],[%.7f,%.7f],[%.7f,%.7f],[%.7f,%.7f]]", \
        lat-0.004,lon-0.006,lat-0.004,lon+0.006, \
        lat+0.004,lon+0.006,lat+0.004,lon-0.006
}')"

docker_bin="${DOCKER_BIN:-docker}"
airside_image="${AIRSIDE_IMAGE:-warg/airside:latest}"
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
"$docker_bin" image inspect "$airside_image" --format '{{.Id}}' \
    > "$artifact_dir/airside-image-id.txt"
source_commit="$(git -c "safe.directory=$repo_root" -C "$repo_root" rev-parse HEAD 2>/dev/null || true)"
if [[ -z "$source_commit" ]] && command -v git.exe >/dev/null 2>&1; then
    win_repo="$(wslpath -w "$repo_root")"
    source_commit="$(git.exe -c "safe.directory=$win_repo" -C "$win_repo" rev-parse HEAD 2>/dev/null || true)"
fi
printf '%s\n' "${source_commit:-unavailable}" > "$artifact_dir/source-commit.txt"
sitl_name="sitl-232-moving-$$"
airside_name="airside-232-moving-$$"
echo "Artifacts: $artifact_dir"

cleanup() {
    "$docker_bin" logs "$sitl_name" > "$artifact_dir/sitl.log" 2>&1 || true
    "$docker_bin" rm -f "$sitl_name" "$airside_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

"$docker_bin" run -d --name "$sitl_name" --network host \
    -e SITL_HOME="$sitl_home" \
    -e SITL_PARAM_FILE="$sitl_param_file" \
    -v "$script_dir":/test:ro \
    warg/sitl:latest bash -lc \
    'cd /ardupilot && home_options=(); \
     if [[ -n "$SITL_HOME" ]]; then home_options=(--home "$SITL_HOME"); fi; \
     exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 "${home_options[@]}" \
     --sim-address=127.0.0.1 \
     --defaults "Tools/autotest/default_params/copter.parm,/test/$SITL_PARAM_FILE"' \
    > "$artifact_dir/sitl-id.txt"
sleep 3

"$docker_bin" run --rm --name "$airside_name" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-220}" \
    -e ROS_LOCALHOST_ONLY=1 \
    -e SCENARIO="$scenario" \
    -e CAPTURE_FILE="$capture_in_container" \
    -e ROUTE_FILE="$route_in_container" \
    -e OWN_AIRCRAFT_INDEX="$own_index" \
    -e REPLAY_START_S="$replay_start_s" \
    -e AEAC_CONNECTION_TOKEN=local-test-token \
    -e AEAC_OWN_AIRCRAFT_INDEX=1 \
    -e AEAC_WEBSOCKET_URL=ws://127.0.0.1:18765 \
    -e FLIGHT_BOUNDARY_WGS84="$flight_boundary" \
    -v "$repo_root":/repo:ro \
    -v "$artifact_dir":/artifacts \
    "$airside_image" bash -lc '
        source /opt/ros/humble/setup.bash
        source /ros_ws/install/setup.bash
        if [[ "$SCENARIO" == listener-* || "$SCENARIO" == engine-* ]]; then
            use_engine=false
            if [[ "$SCENARIO" == engine-* ]]; then use_engine=true; fi
            use_traffic_listener=true
            if [[ "$SCENARIO" == engine-replay ]]; then use_traffic_listener=false; fi
            route_args=()
            if [[ -n "$ROUTE_FILE" ]]; then route_args=(waypoints_file:="$ROUTE_FILE"); fi
            ros2 launch airside_bringup airside.launch.py \
                use_camera:=false \
                use_engine:="$use_engine" \
                use_traffic_listener:="$use_traffic_listener" \
                lap_count:=1 "${route_args[@]}" \
                fcu_url:=tcp://127.0.0.1:5760 \
                > /artifacts/launch.log 2>&1 &
            stack_pid=$!
        else
            ros2 run mavros mavros_node --ros-args \
                -p fcu_url:=tcp://127.0.0.1:5760 \
                -p fcu_protocol:=v2.0 \
                -p tgt_system:=1 -p tgt_component:=1 \
                > /artifacts/mavros.log 2>&1 &
            mavros_pid=$!
            ros2 run navigation position_controller \
                > /artifacts/controller.log 2>&1 &
            controller_pid=$!
        fi
        setsid ros2 bag record -o /artifacts/rosbag \
            /position_controller/target /position_controller/avoidance_status \
            /position_controller/waypoint_adjustment \
            /engine/waypoints \
            /aeac/traffic /position_controller/obstacle \
            /mavros/state /mavros/local_position/pose \
            /mavros/global_position/global /mavros/global_position/rel_alt \
            /mavros/setpoint_raw/global /mavros/statustext/send \
            > /artifacts/rosbag.log 2>&1 &
        bag_pid=$!
        if [[ "$SCENARIO" == engine-replay ]]; then
            timeout 420 python3 /repo/airside/experiments/obstacle_avoidance/sitl_engine_replay.py \
                --capture "$CAPTURE_FILE" --route "$ROUTE_FILE" \
                --own-aircraft-index "$OWN_AIRCRAFT_INDEX" \
                --replay-start-s "$REPLAY_START_S" \
                --summary /artifacts/summary.json
        elif [[ "$SCENARIO" == replay ]]; then
            timeout 420 python3 /repo/airside/experiments/obstacle_avoidance/sitl_server_replay.py \
                --capture "$CAPTURE_FILE" --route "$ROUTE_FILE" \
                --own-aircraft-index "$OWN_AIRCRAFT_INDEX" \
                --replay-start-s "$REPLAY_START_S" \
                --summary /artifacts/summary.json
        elif [[ "$SCENARIO" == listener-* ]]; then
            timeout 420 python3 /repo/airside/experiments/obstacle_avoidance/sitl_moving_readiness.py \
                --scenario "${SCENARIO#listener-}" --traffic-source listener \
                --summary /artifacts/summary.json
        elif [[ "$SCENARIO" == engine-* ]]; then
            timeout 650 python3 /repo/airside/experiments/obstacle_avoidance/sitl_engine_moving.py \
                --route "$ROUTE_FILE" --scenario "$SCENARIO" \
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
        if [[ "$SCENARIO" == listener-* || "$SCENARIO" == engine-* ]]; then
            kill -TERM "$stack_pid" >/dev/null 2>&1 || true
        else
            kill -TERM "$controller_pid" "$mavros_pid" >/dev/null 2>&1 || true
        fi
        exit "$result"
    ' > "$artifact_dir/runner.log" 2>&1 || {
        cat "$artifact_dir/runner.log"
        exit 1
    }

cat "$artifact_dir/summary.json"
