#!/usr/bin/env bash
# Run AEAC traffic controls and the strict ten-attempt moving-traffic campaign.
set -uo pipefail

cd "$(dirname "$0")"

artifact_dir="${ARTIFACT_DIR:?ARTIFACT_DIR is required}"
protocol_fixture="${AEAC_PROTOCOL_FIXTURE:?AEAC_PROTOCOL_FIXTURE is required}"
protocol_metadata="${AEAC_PROTOCOL_METADATA:?AEAC_PROTOCOL_METADATA is required}"
mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"
export ARTIFACT_DIR="$artifact_dir"

if compgen -G "$artifact_dir/*-summary.json" >/dev/null; then
    echo "artifact directory already contains scenario summaries: $artifact_dir" >&2
    exit 2
fi

PYTHONPATH="../../src/aeac_bridge${PYTHONPATH:+:$PYTHONPATH}" \
    python3 ./verify_aeac_protocol_fixture.py \
    "$protocol_fixture" "$protocol_metadata" \
    --output "$artifact_dir/aeac-protocol-verification.json" || exit 2
cp "$protocol_fixture" "$artifact_dir/aeac-traffic-event.sanitized.json"
cp "$protocol_metadata" "$artifact_dir/aeac-protocol-metadata.json"

suite_status=0
for scenario in \
    traffic_clear \
    traffic_static \
    traffic_crossing \
    traffic_dropout \
    traffic_malformed \
    traffic_reconnect \
    traffic_unknown_identity \
    traffic_pilot_takeover; do
    if ! bash ./run_airside_bt_scenario.sh "$scenario" "$scenario"; then
        suite_status=1
    fi
done

for run_number in $(seq -w 1 10); do
    if ! bash ./run_airside_bt_scenario.sh \
        traffic_crossing "traffic-crossing-run-${run_number}"; then
        suite_status=1
    fi
done

python3 - "$artifact_dir" <<'PY'
import json
import pathlib
import sys

artifact_dir = pathlib.Path(sys.argv[1])
control_names = (
    "traffic_clear",
    "traffic_static",
    "traffic_crossing",
    "traffic_dropout",
    "traffic_malformed",
    "traffic_reconnect",
    "traffic_unknown_identity",
    "traffic_pilot_takeover",
)
expected_paths = [artifact_dir / f"{name}-summary.json" for name in control_names]
expected_paths += [
    artifact_dir / f"traffic-crossing-run-{run_number:02d}-summary.json"
    for run_number in range(1, 11)
]
missing = [str(path) for path in expected_paths if not path.is_file()]
if missing:
    print(json.dumps({"missing_summaries": missing}, indent=2))
    raise SystemExit(1)
summary_entries = [
    (path, json.loads(path.read_text(encoding="utf-8")))
    for path in expected_paths
]
summaries = [item for _path, item in summary_entries]
strict_attempts = [
    item
    for path, item in summary_entries
    if path.name.startswith("traffic-crossing-run-")
]
aggregate = {
    "scenario_count": len(summaries),
    "pass_count": sum(item["verdict"] == "PASS" for item in summaries),
    "strict_attempt_count": len(strict_attempts),
    "strict_pass_count": sum(
        item["verdict"] == "PASS" for item in strict_attempts
    ),
    "strict_traffic_10_of_10": (
        len(strict_attempts) == 10
        and all(item["verdict"] == "PASS" for item in strict_attempts)
    ),
    "controls_all_passed": all(
        item["verdict"] == "PASS"
        for path, item in summary_entries
        if path.name in {f"{name}-summary.json" for name in control_names}
    ),
    "summaries": summaries,
}
(artifact_dir / "airside-traffic-aggregate.json").write_text(
    json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(aggregate, indent=2, sort_keys=True))
if not aggregate["strict_traffic_10_of_10"] or not aggregate["controls_all_passed"]:
    raise SystemExit(1)
PY
aggregate_status=$?
if [[ "$aggregate_status" -ne 0 ]]; then
    suite_status=1
fi

exit "$suite_status"
