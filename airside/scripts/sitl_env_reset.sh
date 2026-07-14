#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

REBUILD=false
DAEMON=false
for arg in "$@"; do
  case "$arg" in
    --rebuild) REBUILD=true ;;
    --daemon) DAEMON=true ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

echo "[reset] docker compose down..."
docker compose down --remove-orphans --timeout 20 || true

STRAY=$(docker ps -aq --filter "name=airside" --filter "name=sitl" 2>/dev/null || true)
if [[ -n "${STRAY}" ]]; then
  echo "[reset] removing stray containers: ${STRAY}"
  docker rm -f ${STRAY} || true
fi

if $DAEMON; then
  echo "[reset] restarting the Docker daemon..."
  if command -v snap >/dev/null 2>&1 && snap list docker >/dev/null 2>&1; then
    snap restart docker
  elif command -v systemctl >/dev/null 2>&1; then
    systemctl restart docker
  else
    service docker restart
  fi
  echo -n "[reset] waiting for the daemon"
  for i in $(seq 1 60); do
    if docker info >/dev/null 2>&1; then echo " ... up"; break; fi
    echo -n "."
    sleep 2
  done
  docker info >/dev/null 2>&1 || { echo "daemon did not come back" >&2; exit 1; }
fi

if $REBUILD; then
  echo "[reset] rebuilding the airside image (SITL image stays cached)..."
  docker compose build airside
fi

echo "[reset] done. Bring up with e.g.:"
echo "  AIRSIDE_AUTO_ARM=true AIRSIDE_WORLD_TARGET=true docker compose up -d"
echo "Give the SITL EKF ~90 s to settle before trusting arm/takeoff behaviour."
