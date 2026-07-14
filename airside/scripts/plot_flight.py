import csv
import math
import os
import sys


def _f(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    art_dir = sys.argv[1]
    csv_path = os.path.join(art_dir, "flight.csv")
    rows = [r for r in csv.DictReader(open(csv_path, newline="")) if r["t"]]
    if not rows:
        print("no telemetry rows")
        sys.exit(2)

    t0 = _f(rows[0]["t"])
    t = [_f(r["t"]) - t0 for r in rows]
    z = [_f(r["z"]) for r in rows]
    rng = [
        math.sqrt(_f(r["target_x"], 0) ** 2 + _f(r["target_y"], 0) ** 2 + _f(r["target_z"], 0) ** 2)
        if r["target_z"]
        else math.nan
        for r in rows
    ]
    sp_vx = [_f(r["sp_vx"]) for r in rows]
    hold_z = [_f(r["hold_z"]) for r in rows]
    hold_fresh = [bool(r["hold_age_s"]) and _f(r["hold_age_s"], 9e9) < 1.0 for r in rows]
    armed = [r["armed"] == "1" for r in rows]
    modes = [r["mode"] for r in rows]

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_r, ax_z, ax_v, ax_m) = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    fig.suptitle(f"SITL trial: {os.path.basename(os.path.abspath(art_dir))}")

    ax_r.plot(t, rng, "b-", lw=1)
    ax_r.axhline(2.5, color="orange", ls="--", label="standoff")
    ax_r.axhline(1.5, color="red", label="hard-min")
    ax_r.set_ylabel("range (m)")
    ax_r.legend(fontsize=8)
    ax_r.grid(True)

    ax_z.plot(t, z, "b-", lw=1, label="altitude")
    latched = [hz for hz, fresh in zip(hold_z, hold_fresh) if fresh and not math.isnan(hz)]
    if latched:
        zref = latched[0]
        ax_z.axhline(zref, color="magenta", ls="--", label=f"latched z={zref:.2f}")
        ax_z.axhspan(zref - 0.3, zref + 0.3, color="magenta", alpha=0.08, label="+/-0.30 m")
    for i in range(len(t) - 1):
        if hold_fresh[i]:
            ax_z.axvspan(t[i], t[i + 1], color="magenta", alpha=0.05)
    ax_z.set_ylabel("z (m, ENU)")
    ax_z.legend(fontsize=8)
    ax_z.grid(True)

    ax_v.plot(t, sp_vx, "c-", lw=1, label="commanded forward (FLU x)")
    ax_v.axhline(0, color="k", lw=0.5)
    ax_v.set_ylabel("m/s")
    ax_v.legend(fontsize=8)
    ax_v.grid(True)

    mode_names = sorted(set(modes))
    mode_idx = [mode_names.index(m) for m in modes]
    ax_m.step(t, mode_idx, where="post", lw=1.2, label="mode")
    ax_m.set_yticks(range(len(mode_names)))
    ax_m.set_yticklabels(mode_names, fontsize=8)
    ax_m.step(t, [len(mode_names) - 0.5 if a else -0.5 for a in armed],
              where="post", lw=0.8, color="green", alpha=0.5, label="armed")
    ax_m.set_xlabel("t (s)")
    ax_m.legend(fontsize=8)
    ax_m.grid(True)

    out = os.path.join(art_dir, "flight.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
