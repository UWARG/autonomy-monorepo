#!/usr/bin/env bash
# Run from host (needs docker/sudo) or paste into an already-open jetson container shell.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "=== host: restart jetson stack ==="
sudo docker compose --profile jetson down --remove-orphans
sudo docker compose --profile jetson up --force-recreate
