#!/usr/bin/env bash

SITL_PLUS_DIR="/app/src"
ARDUPILOT_DIR="/app/src/ardupilot"
LOG_DIR="/app/logs"

mkdir -p "$LOG_DIR"
export PYTHONUNBUFFERED=1

echo "[entrypoint] Logs: ${LOG_DIR}/sim_vehicle.log (MAVProxy/SITL), ${LOG_DIR}/pybullet.log"

echo "[entrypoint] Starting PyBullet physics (main.py)..."
cd "$SITL_PLUS_DIR"
uv run python3 main.py > "${LOG_DIR}/pybullet.log" 2>&1 &
MAIN_PID=$!


sleep 10

echo "[entrypoint] Starting ArduPilot SITL (sim_vehicle.py)..."
cd "$ARDUPILOT_DIR"


LAT=-35.362938
LON=149.165085
ALT=584.0805053710938
DIR=270

source /home/devuser/venv-ardupilot/bin/activate
# --out must come before --mavproxy-args (otherwise sim_vehicle can glue --out into mavproxy args)
python3 -u ./Tools/autotest/sim_vehicle.py -N -v ArduCopter \
-f quad --model JSON:127.0.0.1 -w \
--out tcpin:0.0.0.0:5761 \
--mavproxy-args "--moddebug=3 --show-errors --state-basedir=${LOG_DIR}" \
--custom-location=${LAT},${LON},${ALT},${DIR} \
2>&1 > "${LOG_DIR}/sim_vehicle.log" &
SITL_PID=$!


wait -n $MAIN_PID
EXIT_CODE=$?

echo "[entrypoint] Stopping processes..."
kill "$MAIN_PID" "$SITL_PID" 2>/dev/null || true
exit $EXIT_CODE
