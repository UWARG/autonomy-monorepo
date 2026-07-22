"""Assert no setpoints occur beyond one streamer tick after an authority release."""

from __future__ import annotations

import argparse
import csv
import sys


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    trigger = parser.add_mutually_exclusive_group(required=True)
    trigger.add_argument("--mode")
    trigger.add_argument("--reason")
    parser.add_argument("--stream-hz", type=float, default=20.0)
    args = parser.parse_args()
    with open(args.csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    if args.mode:
        trigger_row = next(
            (
                row
                for row in rows
                if row["event"] == "mode_transition" and row["mode"] == args.mode
            ),
            None,
        )
    else:
        trigger_row = next(
            (
                row
                for row in rows
                if row["event"] == "diagnostic" and row["stop_reason"] == args.reason
            ),
            None,
        )
    if trigger_row is None:
        print("RELEASE ASSERTION: FAIL (trigger not recorded)")
        sys.exit(1)

    deadline = float(trigger_row["host_time_s"]) + 1.0 / args.stream_hz + 0.01
    late = [
        row
        for row in rows
        if row["event"] == "setpoint" and float(row["host_time_s"]) > deadline
    ]
    print(
        f"RELEASE ASSERTION: {'PASS' if not late else 'FAIL'} "
        f"(late setpoints={len(late)}, deadline={deadline:.6f})"
    )
    sys.exit(0 if not late else 1)


if __name__ == "__main__":
    main()
