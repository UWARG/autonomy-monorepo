"""
Small tester for cluster_estimation.

    python cluster_tester.py gen                    # write sample_points.txt (3D, tagged)
    python cluster_tester.py run sample_points.txt              # cluster + print
    python cluster_tester.py run sample_points.txt -g color     # cluster per tag
    python cluster_tester.py run sample_points.txt -g color -p -w
        -p / --plot    save a scatter plot (plots/)
        -w / --write   save cluster locations (out/)
"""

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "..", "src")))

from cluster_estimation import (
    cluster_estimation, cluster_by_tag, bucket_points_by_tag,
    load_points_from_file, write_cluster_locations,
    write_grouped_cluster_locations,
)

import argparse
import os
import sys

# --- generate ---------------------------------------------------------------

def gen(path, spread=0.6, seed=7):
    """Write a 3D tagged sample file: Red has 2 clusters, Blue has 1."""
    import numpy as np
    rng = np.random.default_rng(seed)
    groups = [("Red", [(0, 0, 0), (40, 0, 5)], 10),
              ("Blue", [(20, 0, 10)], 10)]
    lines = ["# x y z | color=..."]
    for color, centers, n in groups:
        for c in centers:
            for _ in range(n):
                p = [v + rng.normal(0, spread) for v in c]
                lines.append(" ".join(f"{v:.3f}" for v in p) + f" | color={color}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {path} ({len(lines)-1} points)")


# --- plot -------------------------------------------------------------------

def plot(name, buckets, results, dim, outdir):
    """Scatter points by bucket, cluster centers as X. 2D/3D only."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    if dim not in (2, 3):
        print(f"  (no plot: {dim}D)")
        return
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d") if dim == 3 else fig.add_subplot(111)
    cmap = plt.get_cmap("tab10")

    for i, (tag, pts) in enumerate(sorted(buckets.items())):
        col = cmap(i % 10)
        a = np.asarray(pts, dtype=float)
        ax.scatter(*a.T[:dim], color=col, s=25, alpha=0.7, label=str(tag))
        for mean, _, _ in (results.get(tag) or []):
            ax.scatter(*[[m] for m in mean[:dim]], color=col, marker="X",
                       s=220, edgecolors="black", linewidths=1.5)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    if dim == 3:
        ax.set_zlabel("z")
    ax.set_title(name)
    ax.legend(loc="best", fontsize=8)
    os.makedirs(outdir, exist_ok=True)
    out = os.path.join(outdir, f"{name}.png")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  plot -> {out}")


# --- run --------------------------------------------------------------------

def run(path, key=None, do_plot=False, do_write=False,
        outdir="plots", write_dir="out"):
    points, tags = load_points_from_file(path, with_tags=True)
    dim = len(points[0])
    name = os.path.splitext(os.path.basename(path))[0]
    print(f"{os.path.basename(path)}: {len(points)} points ({dim}D)")

    if key:
        results = cluster_by_tag(points, tags, key)
        buckets = bucket_points_by_tag(points, tags, key)
        for tag, clusters in sorted(results.items()):
            print(f"  {key}={tag}: {len(clusters or [])} cluster(s)")
            for mean, w, _ in (clusters or []):
                coords = ", ".join(f"{v:7.3f}" for v in mean)
                print(f"      ({coords})  weight={w:.3f}")
        if do_write:
            os.makedirs(write_dir, exist_ok=True)
            out = os.path.join(write_dir, f"{name}_by_{key}.txt")
            write_grouped_cluster_locations(results, out)
            print(f"  wrote -> {out}")
        if do_plot:
            plot(f"{name}_by_{key}", buckets, results, dim, outdir)
    else:
        clusters = cluster_estimation(points)
        print(f"  {len(clusters)} cluster(s)")
        for mean, w, _ in clusters:
            coords = ", ".join(f"{v:7.3f}" for v in mean)
            print(f"      ({coords})  weight={w:.3f}")
        if do_write:
            os.makedirs(write_dir, exist_ok=True)
            out = os.path.join(write_dir, f"{name}.txt")
            write_cluster_locations(clusters, out)
            print(f"  wrote -> {out}")
        if do_plot:
            plot(name, {"all": points}, {"all": clusters}, dim, outdir)


def main():
    ap = argparse.ArgumentParser(description="Test cluster_estimation.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("gen", help="generate a sample point file")
    g.add_argument("path", nargs="?", default=os.path.join(_HERE, "sample_points.txt"))

    r = sub.add_parser("run", help="cluster a point file")
    r.add_argument("path")
    r.add_argument("-g", "--group-by", metavar="KEY", help="cluster per KEY=value tag")
    r.add_argument("-p", "--plot", action="store_true")
    r.add_argument("-w", "--write", action="store_true")
    r.add_argument("--outdir", default="plots")
    r.add_argument("--write-dir", default="out")

    a = ap.parse_args()
    if a.cmd == "gen":
        gen(a.path)
    else:
        run(a.path, a.group_by, a.plot, a.write, a.outdir, a.write_dir)


if __name__ == "__main__":
    main()
