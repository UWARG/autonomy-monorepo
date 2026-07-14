#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")/.."

TS=$(date +%Y%m%d_%H%M%S)
ART_REL="test_artifacts/${1:-hold_${TS}}"
ART_HOST="$(pwd)/${ART_REL}"
ART_CONT="/monorepo/airside/${ART_REL}"
mkdir -p "${ART_HOST}"
WINDOW=120
LATCH_TIMEOUT=300   # EKF settle (~90 s) + approach + convergence

echo "[trial] artifacts -> ${ART_REL}"
docker compose down --remove-orphans --timeout 20 >/dev/null 2>&1 || true

echo "[trial] compose up (AUTO_ARM + WORLD_TARGET)..."
AIRSIDE_AUTO_ARM=true AIRSIDE_WORLD_TARGET=true docker compose up -d || exit 2

echo "[trial] starting recorder + MCAP bag in the container..."
docker compose exec -d airside bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   python3 /monorepo/airside/scripts/flight_recorder.py --out ${ART_CONT}/flight.csv" || exit 2
docker compose exec -d airside bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   ros2 bag record -s mcap -o ${ART_CONT}/bag \
     /mavros/local_position/pose /mavros/state /mavros/setpoint_raw/local \
     /mavros/setpoint_position/local /perception/target /viz/markers /viz/range /viz/state" \
  || echo "[trial] (MCAP recording unavailable; continuing with CSV only)"

echo -n "[trial] waiting for 'position hold engaged' (max ${LATCH_TIMEOUT}s)"
LATCHED=false
for i in $(seq 1 $((LATCH_TIMEOUT / 5))); do
  if docker compose logs airside 2>/dev/null | grep -q "position hold engaged"; then
    LATCHED=true
    echo " ... latched"
    break
  fi
  echo -n "."
  sleep 5
done
if ! $LATCHED; then
  echo " TIMEOUT"
  docker compose logs airside > "${ART_HOST}/airside.log" 2>&1
  docker compose down --timeout 20 >/dev/null 2>&1
  echo "HOLD TRIAL: FAIL (hold never engaged; see ${ART_REL}/airside.log)"
  exit 1
fi

echo "[trial] recording a ${WINDOW}s hold window..."
sleep "${WINDOW}"

docker compose logs airside > "${ART_HOST}/airside.log" 2>&1
docker compose down --timeout 20 >/dev/null 2>&1

python3 scripts/evaluate_hold.py "${ART_HOST}" --window "${WINDOW}"
