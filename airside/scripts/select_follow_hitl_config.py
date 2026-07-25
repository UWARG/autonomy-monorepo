"""Select 20 Hz or 10 Hz inference only from completed hardware gates."""

from __future__ import annotations

import argparse
import json
import sys


def select_configuration(
    timing_20: dict,
    timing_10: dict,
    xyz_gate: dict,
) -> dict:
    candidates = []
    for inference_hz, detector_stride, timing in (
        (20, 1, timing_20),
        (10, 2, timing_10),
    ):
        vmax = timing.get("selected_vmax_mps")
        eligible = (
            bool(xyz_gate.get("calibration_gate_pass"))
            and bool(timing.get("gate_5_pass"))
            and bool(timing.get("checks", {}).get("crossing_identity_retained"))
            and bool(timing.get("checks", {}).get("proximity_capture_to_zero_recorded"))
            and vmax is not None
            and float(vmax) >= 0.5
        )
        if eligible:
            candidates.append(
                {
                    "inference_hz": inference_hz,
                    "camera_fps": 20,
                    "detector_stride": detector_stride,
                    "selected_vmax_mps": float(vmax),
                    "reaction_time_s": float(timing["reaction_time_s"]),
                    "target_freshness_s": float(timing["target_freshness_s"]),
                    "stream_hz": float(timing["stream_hz"]),
                    "ema_alpha": float(timing["ema_alpha"]),
                }
            )
    # Highest safe speed wins. The negative inference rate makes 10 Hz win an
    # exact speed tie, but only after all accuracy/identity/stop gates passed.
    selected = max(
        candidates,
        key=lambda item: (
            item["selected_vmax_mps"],
            -item["inference_hz"],
        ),
        default=None,
    )
    return {
        "configuration_selection_pass": selected is not None,
        "selected": selected,
        "eligible_configurations": candidates,
        "rule": (
            "highest passing v_max; exact ties choose 10 Hz only after XYZ, "
            "identity-retention, and proximity-stop timing gates pass"
        ),
    }


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timing-20", required=True)
    parser.add_argument("--timing-10", required=True)
    parser.add_argument("--xyz-gate", required=True)
    parser.add_argument("--json-out", required=True)
    args = parser.parse_args()
    result = select_configuration(
        _load(args.timing_20),
        _load(args.timing_10),
        _load(args.xyz_gate),
    )
    with open(args.json_out, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    sys.exit(0 if result["configuration_selection_pass"] else 1)


if __name__ == "__main__":
    main()
