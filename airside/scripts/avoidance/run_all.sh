#!/usr/bin/env bash
# Run the full issue-96 avoidance scenario suite, one fresh SITL boot each.
# Usage: ./run_all.sh [scenario ...]
# With no arguments, runs every scenario. Needs docker + warg/sitl:latest.
set -u
cd "$(dirname "$0")"
mkdir -p logs
if [ "$#" -eq 0 ]; then
    : > logs/summaries.txt
else
    touch logs/summaries.txt
fi
planner_src="$(cd ../../../obstacle-avoidance && pwd)/src"
scenarios="${*:-clear_guided wall_guided wall_guided_wpnav wall_guided_vel wall_auto wall_custom_2d}"
suite_status=0

for s in $scenarios; do
    docker rm -f sitl-96 >/dev/null 2>&1
    docker run -d --name sitl-96 \
        -v "$PWD":/demo \
        -v "$planner_src":/planner:ro \
        -e PYTHONPATH=/planner \
        warg/sitl:latest bash -lc \
        'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
         --sim-address=127.0.0.1 \
         --defaults Tools/autotest/default_params/copter.parm,/demo/sitl_avoidance.parm' \
        >/dev/null
    sleep 3
    duration="${DEMO_DURATION_S:-90}"
    if [ -z "${DEMO_DURATION_S:-}" ] && [ "$s" = "wall_guided_vel" ]; then
        duration=45  # never reaches the goal (it stops); cap the watch window
    fi
    console_log="logs/${s}_console.log"
    docker exec sitl-96 python3 /demo/avoidance_demo.py \
        --scenario "$s" --duration "$duration" >"$console_log" 2>&1
    demo_status=$?
    cat "$console_log"
    grep -v "\"scenario\": \"$s\"" logs/summaries.txt \
        > logs/summaries.tmp || true
    mv logs/summaries.tmp logs/summaries.txt
    grep 'summary:' "$console_log" | tee -a logs/summaries.txt
    if [ "$demo_status" -ne 0 ]; then
        suite_status=1
    fi
done

docker rm -f sitl-96 >/dev/null 2>&1
echo "suite complete -> logs/summaries.txt"
exit "$suite_status"
