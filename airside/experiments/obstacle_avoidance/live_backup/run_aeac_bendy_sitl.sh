#!/usr/bin/env bash
# Mock AEAC WebSocket -> live listener -> BendyRuler2D -> ArduCopter SITL.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../../.." && pwd)"
docker_bin="${DOCKER_BIN:-docker}"
airside_image="${AIRSIDE_IMAGE:-warg/airside:pr181-bendy}"
artifact_root="${ARTIFACT_DIR:-$repo_root/airside/log/aeac-bendy-sitl}"

for attempt in {1..12}; do
    if "$docker_bin" info >/dev/null 2>&1; then break; fi
    sleep 3
done
"$docker_bin" info >/dev/null
mkdir -p "$artifact_root"
artifact_dir="$(mktemp -d "$(realpath "$artifact_root")/run-XXXXXXXX")"
echo "Artifacts: $artifact_dir"

scenario="${1:-traffic_static}"
case "$scenario" in
    traffic_clear|traffic_static|traffic_dropout|traffic_pilot_takeover) ;;
    *) echo "Unknown scenario: $scenario" >&2; exit 2 ;;
esac

sitl_name="sitl-181-bendy-$scenario"
airside_name="airside-181-bendy-$scenario"
if [[ -n "$("$docker_bin" ps -a --filter "name=^/$sitl_name$" --format '{{.Names}}')" \
   || -n "$("$docker_bin" ps -a --filter "name=^/$airside_name$" --format '{{.Names}}')" ]]; then
    echo "Test container name already exists; refusing to replace it" >&2
    exit 2
fi
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

"$docker_bin" run --rm --name "$airside_name" --network host --ipc host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-181}" \
    -e ROS_LOCALHOST_ONLY=1 \
    -e SCENARIO="$scenario" \
    -v "$repo_root":/repo:ro \
    -v "$artifact_dir":/artifacts \
    "$airside_image" \
    bash /repo/airside/experiments/obstacle_avoidance/live_backup/run_aeac_bendy_container.sh \
    > "$artifact_dir/$scenario-runner.log" 2>&1 || result=$?

result="${result:-0}"
cat "$artifact_dir/$scenario-runner.log"
echo "$result" > "$artifact_dir/$scenario-exit.txt"
exit "$result"
