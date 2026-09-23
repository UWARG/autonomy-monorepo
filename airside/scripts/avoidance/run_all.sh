#!/usr/bin/env bash
# Run issue-96 avoidance scenarios, one fresh SITL container per scenario.
# Formal callers set ARTIFACT_DIR, RUN_LABEL, and SKIP_HARNESS_TESTS=1.
set -u

cd "$(dirname "$0")"

if [ "${SKIP_HARNESS_TESTS:-0}" != "1" ]; then
    python3 -m unittest discover -v -s . -p 'test_*.py' || exit 1
fi

artifact_dir="${ARTIFACT_DIR:-$PWD/logs}"
mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"
planner_src="$(cd ../../../obstacle-avoidance && pwd)/src"
scenarios="${*:-clear_guided wall_guided wall_guided_wpnav wall_guided_vel wall_auto wall_custom_2d}"
suite_status=0

cleanup_container() {
    docker rm -f sitl-96 >/dev/null 2>&1 || true
}
trap cleanup_container EXIT
trap 'cleanup_container; exit 130' INT TERM

for scenario in $scenarios; do
    label="${RUN_LABEL:-$scenario}"
    runner_log="$artifact_dir/${label}-runner.log"
    fc_log="$artifact_dir/${label}-fc.log"
    exit_file="$artifact_dir/${label}-exit.txt"
    inspect_file="$artifact_dir/${label}-container-inspect.json"

    cleanup_container
    if ! docker run -d --name sitl-96 \
        -v "$PWD":/demo:ro \
        -v "$planner_src":/planner:ro \
        -v "$artifact_dir":/artifacts \
        -e PYTHONPATH=/planner \
        -e MAVLINK20=1 \
        warg/sitl:latest bash -lc \
        'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
         --sim-address=127.0.0.1 \
         --defaults Tools/autotest/default_params/copter.parm,/demo/sitl_avoidance.parm' \
        >/dev/null; then
        echo "docker run failed" >"$runner_log"
        echo 125 >"$exit_file"
        suite_status=1
        continue
    fi

    docker inspect sitl-96 >"$inspect_file" 2>&1 || true
    sleep 3
    duration="${DEMO_DURATION_S:-90}"
    if [ -z "${DEMO_DURATION_S:-}" ] && [ "$scenario" = "wall_guided_vel" ]; then
        duration=45
    fi

    docker exec -e MAVLINK20=1 sitl-96 python3 /demo/avoidance_demo.py \
        --scenario "$scenario" \
        --duration "$duration" \
        --log "/artifacts/${label}.jsonl" \
        --summary-json "/artifacts/${label}-summary.json" \
        --params-json "/artifacts/${label}-params.json" \
        >"$runner_log" 2>&1
    demo_status=$?
    echo "$demo_status" >"$exit_file"
    docker logs sitl-96 >"$fc_log" 2>&1 || true
    cat "$runner_log"
    if [ "$demo_status" -ne 0 ]; then
        suite_status=1
    fi
done

cleanup_container
echo "suite complete -> $artifact_dir"
exit "$suite_status"
