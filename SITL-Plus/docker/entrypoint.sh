#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  echo "[entrypoint] Caught stop signal, shutting down..."
  if [[ -n "${SITL_PID:-}" ]]; then
    kill "$SITL_PID" 2>/dev/null || true
  fi
  if [[ -n "${MAIN_PID:-}" ]]; then
    kill "$MAIN_PID" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
  exit 0
}
trap cleanup SIGTERM SIGINT

SITL_PLUS_DIR="/app"
PY_SRC_DIR="/app/src"
ARDUPILOT_DIR="/app/ardupilot"

echo "[entrypoint] Starting PyBullet physics (main.py)..."
cd "$PY_SRC_DIR"
uv run python main.py --nogui &
MAIN_PID=$!

sleep 2

echo "[entrypoint] Starting ArduPilot SITL (sim_vehicle.py)..."
cd "$ARDUPILOT_DIR"
python ./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1  --console --map --out tcpin:0.0.0.0:5761 &
SITL_PID=$!

wait -n $MAIN_PID $SITL_PID
EXIT_CODE=$?

echo "[entrypoint] Stopping processes..."
kill "$MAIN_PID" "$SITL_PID" 2>/dev/null || true
exit $EXIT_CODE
