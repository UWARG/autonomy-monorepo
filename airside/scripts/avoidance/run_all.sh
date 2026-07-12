#!/usr/bin/env bash
# Run the full issue-96 avoidance scenario suite, one fresh SITL boot each.
# Usage: ./run_all.sh   (from this directory; needs docker + warg/sitl:latest)
set -u
cd "$(dirname "$0")"
mkdir -p logs
: > logs/summaries.txt

for s in clear_guided wall_guided wall_guided_wpnav wall_guided_vel wall_auto; do
    docker rm -f sitl-96 >/dev/null 2>&1
    docker run -d --name sitl-96 -v "$PWD":/demo warg/sitl:latest bash -lc \
        'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
         --sim-address=127.0.0.1 \
         --defaults Tools/autotest/default_params/copter.parm,/demo/sitl_avoidance.parm' \
        >/dev/null
    sleep 3
    duration=90
    if [ "$s" = "wall_guided_vel" ]; then
        duration=45  # never reaches the goal (it stops); cap the watch window
    fi
    docker exec sitl-96 python3 /demo/avoidance_demo.py \
        --scenario "$s" --duration "$duration" 2>&1 \
        | grep 'summary:' | tee -a logs/summaries.txt
done

docker rm -f sitl-96 >/dev/null 2>&1
echo "suite complete -> logs/summaries.txt"
