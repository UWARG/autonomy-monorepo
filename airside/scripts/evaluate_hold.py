import argparse
import csv
import math
import os
import re
import sys

LATCH_RE = re.compile(r"position hold engaged at \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)")
ALT_TOL = 0.30
DRIFT_TOL = 1.0
MAX_LATCHES = 3


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact_dir")
    parser.add_argument("--window", type=float, default=120.0)
    args = parser.parse_args()

    log_path = os.path.join(args.artifact_dir, "airside.log")
    csv_path = os.path.join(args.artifact_dir, "flight.csv")
    if not (os.path.exists(log_path) and os.path.exists(csv_path)):
        print(f"EVALUATE: missing artifacts in {args.artifact_dir}")
        sys.exit(2)

    log = open(log_path, encoding="utf-8", errors="replace").read()
    latches = LATCH_RE.findall(log)
    follow_idx = log.find("follow active")
    flight_log = log[follow_idx:] if follow_idx >= 0 else log
    target_lost = len(re.findall(r"target lost", flight_log))

    checks = {}
    checks["hold engaged"] = (len(latches) >= 1, f"{len(latches)} latch(es)")
    checks[f"latch count <= {MAX_LATCHES}"] = (
        len(latches) <= MAX_LATCHES, f"{len(latches)}"
    )
    checks["zero 'target lost'"] = (target_lost == 0, f"{target_lost} occurrence(s)")

    if latches:
        hold_x, hold_y, hold_z = (float(v) for v in latches[0])
        rows = list(csv.DictReader(open(csv_path, newline="")))
        rows = [r for r in rows if r["t"]]
        start_idx = next(
            (i for i, r in enumerate(rows) if r["hold_age_s"] not in ("", None)), None
        )
        if start_idx is None:
            checks["hold setpoint seen in telemetry"] = (False, "never published")
        else:
            t0 = float(rows[start_idx]["t"])
            window = [
                r for r in rows if t0 <= float(r["t"]) <= t0 + args.window
            ]
            alt_err = max(abs(float(r["z"]) - hold_z) for r in window)
            drift = max(
                math.hypot(float(r["x"]) - hold_x, float(r["y"]) - hold_y) for r in window
            )
            modes = {r["mode"] for r in window}
            disarmed = any(r["armed"] == "0" for r in window)
            dur = float(window[-1]["t"]) - t0 if window else 0.0
            checks[f"window covers >= {args.window:.0f}s"] = (
                dur >= args.window * 0.95, f"{dur:.0f}s"
            )
            checks[f"altitude within +/-{ALT_TOL} m of latched z"] = (
                alt_err <= ALT_TOL, f"max err {alt_err:.2f} m (latched z={hold_z:.2f})"
            )
            checks[f"horizontal drift <= {DRIFT_TOL} m"] = (drift <= DRIFT_TOL, f"{drift:.2f} m")
            checks["GUIDED throughout"] = (modes == {"GUIDED"}, f"modes={sorted(modes)}")
            checks["armed throughout"] = (not disarmed, "disarmed!" if disarmed else "armed")

    ok = all(passed for passed, _ in checks.values())
    print(f"HOLD TRIAL: {'PASS' if ok else 'FAIL'}  ({args.artifact_dir})")
    for name, (passed, detail) in checks.items():
        print(f"  [{'ok' if passed else 'XX'}] {name}: {detail}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
