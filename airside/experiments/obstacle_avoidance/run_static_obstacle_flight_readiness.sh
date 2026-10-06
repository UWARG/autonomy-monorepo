#!/usr/bin/env bash
# Qualify the static imaginary-obstacle path before a controlled flight test.
set -uo pipefail

cd "$(dirname "$0")"

artifact_dir="${ARTIFACT_DIR:?ARTIFACT_DIR is required}"
mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"
export ARTIFACT_DIR="$artifact_dir"

if compgen -G "$artifact_dir/*-summary.json" >/dev/null; then
    echo "artifact directory already contains scenario summaries: $artifact_dir" >&2
    exit 2
fi

suite_status=0
if ! bash ./run_airside_bt_scenario.sh traffic_clear traffic-clear; then
    suite_status=1
fi
if ! bash ./run_airside_bt_scenario.sh \
    traffic_pilot_takeover traffic-pilot-takeover; then
    suite_status=1
fi

# Three fresh boots are enough for this test-readiness gate. This is not a
# competition-server or general moving-traffic qualification campaign.
for run_number in 01 02 03; do
    if ! bash ./run_airside_bt_scenario.sh \
        traffic_static "traffic-static-run-${run_number}"; then
        suite_status=1
    fi
done

python3 - "$artifact_dir" <<'PY'
import json
import pathlib
import sys

artifact_dir = pathlib.Path(sys.argv[1])
names = [
    "traffic-clear",
    "traffic-pilot-takeover",
    "traffic-static-run-01",
    "traffic-static-run-02",
    "traffic-static-run-03",
]
paths = [artifact_dir / f"{name}-summary.json" for name in names]
missing = [str(path) for path in paths if not path.is_file()]
summaries = [
    json.loads(path.read_text(encoding="utf-8"))
    for path in paths
    if path.is_file()
]
static_runs = [item for item in summaries if item["scenario"] == "traffic_static"]
aggregate = {
    "scope": "static imaginary-obstacle flight-test readiness",
    "missing_summaries": missing,
    "scenario_count": len(summaries),
    "pass_count": sum(item["verdict"] == "PASS" for item in summaries),
    "static_attempt_count": len(static_runs),
    "static_pass_count": sum(item["verdict"] == "PASS" for item in static_runs),
    "ready": (
        not missing
        and len(static_runs) == 3
        and all(item["verdict"] == "PASS" for item in summaries)
    ),
    "summaries": summaries,
}
(artifact_dir / "static-obstacle-flight-readiness.json").write_text(
    json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(aggregate, indent=2, sort_keys=True))
if not aggregate["ready"]:
    raise SystemExit(1)
PY
aggregate_status=$?
if [[ "$aggregate_status" -ne 0 ]]; then
    suite_status=1
fi

exit "$suite_status"
