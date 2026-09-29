#!/usr/bin/env bash
# One shared physics process and N isolated ArduPilot processes.
set -euo pipefail
if [[ $# != 1 || ! $1 =~ ^[1-9][0-9]*$ ]] || (( $1 > 100 )); then
    echo "Usage: bash start.sh <drone-count: 1-100>" >&2
    exit 1
fi
cd "$(dirname "${BASH_SOURCE[0]}")"
override=$(mktemp)
trap 'rm -f "$override"' EXIT
export DRONE_HOSTS=sitl-plus
printf 'services:\n' > "$override"
for ((i = 2; i <= $1; i++)); do
    export DRONE_HOSTS="$DRONE_HOSTS,drone-$i"
    cat >> "$override" <<EOF
  drone-$i:
    extends:
      file: docker-compose.yml
      service: sitl-plus
    ports: !override
      - "$((5760 + i)):5761"
    environment:
      GCS_PORT: $((14549 + i))
    volumes: !override
      - ./logs/drone_$i:/app/logs
EOF
done
# A nonempty mapping is needed when no extra controllers were requested.
if (( $1 == 1 )); then
    printf '  physics: {}\n' >> "$override"
fi
docker compose -f docker-compose.yml -f "$override" up --build --remove-orphans
