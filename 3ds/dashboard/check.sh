#!/usr/bin/env bash
# Run from any directory. Install locked dependencies on a fresh checkout.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ! -d dashboard/node_modules ]]; then
  npm --prefix dashboard ci --no-audit --no-fund
fi
npm --prefix dashboard run typecheck
npm --prefix dashboard run lint
npm --prefix dashboard test
export PYTHONPATH="src/dashboard_monitor:src/engine${PYTHONPATH:+:${PYTHONPATH}}"
python3 -m unittest discover -s src/dashboard_monitor/test -v
python3 -m unittest discover -s src/engine/test -p test_telemetry.py -v
python3 -m compileall -q src/dashboard_monitor src/engine/engine/telemetry.py src/airside_bringup
