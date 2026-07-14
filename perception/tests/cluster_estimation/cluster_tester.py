"""
General tester for cluster_estimation.

Usage:
    python cluster_tester.py file1.txt file2.txt ...
    python cluster_tester.py data/*.txt
    python cluster_tester.py --dir path/to/folder
    python cluster_tester.py --dir data --glob "*.csv"
    python cluster_tester.py data/*.txt --visualize
    python cluster_tester.py data/*.txt --visualize --outdir plots
    python cluster_tester.py data/*.txt --group-by color
    python cluster_tester.py data/*.txt --group-by color --visualize
    python cluster_tester.py data/*.txt --group-by color --include-untagged

Each file is loaded and clustered independently. A bad file (missing,
malformed, unclusterable) is reported and skipped -- it won't stop the
others. Exit code is non-zero if any file failed.

--visualize saves a scatter plot per file (2D or 3D), points colored by
their nearest returned cluster center, centers marked with an X.

--group-by KEY clusters each 'KEY=value' tag bucket separately (e.g.
--group-by color makes Red points cluster only with other Red points).
"""

import argparse
import glob
import os
import sys

# --- make src/ importable regardless of where this is run from ---------------
# This file lives at <repo>/tests/cluster_estim/cluster_tester.py
# The module lives at <repo>/src/cluster_estimation.py
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
# -----------------------------------------------------------------------------

from cluster_estimation import (
    cluster_estimation,
    cluster_by_tag,
    bucket_points_by_tag,
    load_points_from_file,
)


def process_file(path):
    """Load and cluster one file. Returns (status, info)."""
    try:
        points, tags = load_points_from_file(path, with_tags=True)
    except FileNotFoundError:
        return "error", "file not found"
    except OSError as e:
        return "error", f"could not read file ({e})"
    except ValueError as e:
        return "error", f"parse error: {e}"

    if not points:
        return "empty", "no points in file"

    dim = len(points[0])

    try:
        clusters = cluster_estimation(points)
    except Exception as e:
        return "error", f"clustering failed: {type(e).__name__}: {e}"

    if clusters is None:
        return "error", "cluster_estimation returned None (bad config)"

    return "ok", {"n_points": len(points), "dim": dim,
                  "clusters": clusters, "points": points, "tags": tags}


def tag_breakdown_per_cluster(points, tags, clusters):
    """Assign each point to its nearest center, tally tags per cluster."""
    from collections import Counter
    centers = [c[0] for c in clusters]
    if not centers:
        return []
    _, labels = assign_to_nearest(points, centers)
    counters = [Counter() for _ in clusters]
    for label, point_tags in zip(labels, tags):
        counters[label].update(point_tags)
    return counters


def print_result(path, status, info):
    name = os.path.basename(path)
    if status == "ok":
        c = info["clusters"]
        print(f"[OK]    {name}: {info['n_points']} points ({info['dim']}D) "
              f"-> {len(c)} cluster(s)")
        has_tags = any(t for t in info.get("tags", []))
        counters = (tag_breakdown_per_cluster(info["points"], info["tags"], c)
                    if has_tags else [None] * len(c))
        for (mean, weight, cov), counter in zip(c, counters):
            coords = ", ".join(f"{v:8.3f}" for v in mean)
            print(f"          center=({coords})  weight={weight:.3f}  cov={cov:.3f}")
            if counter:
                summary = ", ".join(f"{tag} x{n}"
                                    for tag, n in counter.most_common())
                print(f"              tags: {summary}")
    elif status == "empty":
        print(f"[EMPTY] {name}: {info}")
    else:
        print(f"[FAIL]  {name}: {info}")


def print_grouped_result(path, info, key, include_untagged):
    """Print clustering results, one section per tag bucket."""
    name = os.path.basename(path)
    results = cluster_by_tag(info["points"], info["tags"], key, include_untagged)
    if not results:
        print(f"[WARN]  {name}: no points carry a '{key}=' tag")
        return
    print(f"[OK]    {name}: {info['n_points']} points ({info['dim']}D), "
          f"grouped by '{key}' -> {len(results)} bucket(s)")
    for value, clusters in sorted(results.items()):
        clusters = clusters or []
        print(f"        {key}={value}: -> {len(clusters)} cluster(s)")
        for mean, weight, cov in clusters:
            coords = ", ".join(f"{v:8.3f}" for v in mean)
            print(f"            center=({coords})  weight={weight:.3f}  cov={cov:.3f}")


def assign_to_nearest(points, centers):
    """Label each point by the index of its nearest cluster center."""
    import numpy as np
    pts = np.asarray(points, dtype=float)
    ctr = np.asarray(centers, dtype=float)
    dists = np.linalg.norm(pts[:, None, :] - ctr[None, :, :], axis=2)
    return pts, np.argmin(dists, axis=1)


def visualize(path, info, outdir):
    """Save a scatter plot of one file's points colored by cluster."""
    import matplotlib
    matplotlib.use("Agg")  # no display needed; write straight to file
    import matplotlib.pyplot as plt
    import numpy as np

    points = info["points"]
    clusters = info["clusters"]
    dim = info["dim"]
    name = os.path.splitext(os.path.basename(path))[0]

    if dim not in (2, 3):
        print(f"        (skipping plot for {name}: {dim}D not plottable)")
        return None

    centers = [c[0] for c in clusters]
    if centers:
        pts, labels = assign_to_nearest(points, centers)
    else:
        pts, labels = np.asarray(points, dtype=float), np.zeros(len(points), int)

    fig = plt.figure(figsize=(7, 6))
    cmap = plt.get_cmap("tab10")

    if dim == 2:
        ax = fig.add_subplot(111)
        ax.scatter(pts[:, 0], pts[:, 1], c=[cmap(l % 10) for l in labels],
                   s=25, alpha=0.8)
        for i, (mean, _, _) in enumerate(clusters):
            ax.scatter(mean[0], mean[1], c=[cmap(i % 10)], marker="X",
                       s=200, edgecolors="black", linewidths=1.5)
        ax.set_xlabel("x"); ax.set_ylabel("y")
    else:  # dim == 3
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                   c=[cmap(l % 10) for l in labels], s=25, alpha=0.8)
        for i, (mean, _, _) in enumerate(clusters):
            ax.scatter(mean[0], mean[1], mean[2], c=[cmap(i % 10)], marker="X",
                       s=200, edgecolors="black", linewidths=1.5)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")

    ax.set_title(f"{name}: {len(points)} points, {len(clusters)} cluster(s)")
    os.makedirs(outdir, exist_ok=True)
    out_path = os.path.join(outdir, f"{name}.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def visualize_grouped(path, info, key, include_untagged, outdir):
    """Save one scatter plot per file showing the per-bucket clustering.

    Points are colored by their tag bucket; each bucket's cluster centers
    are marked with an X in the same color. A legend maps colors to values.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    dim = info["dim"]
    name = os.path.splitext(os.path.basename(path))[0]

    if dim not in (2, 3):
        print(f"        (skipping plot for {name}: {dim}D not plottable)")
        return None

    buckets = bucket_points_by_tag(info["points"], info["tags"], key, include_untagged)
    results = cluster_by_tag(info["points"], info["tags"], key, include_untagged)
    if not buckets:
        print(f"        (skipping plot for {name}: no '{key}=' tags)")
        return None

    fig = plt.figure(figsize=(7, 6))
    cmap = plt.get_cmap("tab10")
    ax = (fig.add_subplot(111, projection="3d") if dim == 3
          else fig.add_subplot(111))

    for b_idx, (value, bucket_points) in enumerate(sorted(buckets.items())):
        color = cmap(b_idx % 10)
        pts = np.asarray(bucket_points, dtype=float)
        label = f"{key}={value}"
        if dim == 2:
            ax.scatter(pts[:, 0], pts[:, 1], color=color, s=25, alpha=0.7,
                       label=label)
        else:
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=color, s=25,
                       alpha=0.7, label=label)

        for mean, _, _ in (results.get(value) or []):
            if dim == 2:
                ax.scatter(mean[0], mean[1], color=color, marker="X", s=220,
                           edgecolors="black", linewidths=1.5)
            else:
                ax.scatter(mean[0], mean[1], mean[2], color=color, marker="X",
                           s=220, edgecolors="black", linewidths=1.5)

    ax.set_xlabel("x"); ax.set_ylabel("y")
    if dim == 3:
        ax.set_zlabel("z")
    ax.set_title(f"{name}: grouped by '{key}' ({len(buckets)} bucket(s))")
    ax.legend(loc="best", fontsize=8)

    os.makedirs(outdir, exist_ok=True)
    out_path = os.path.join(outdir, f"{name}__by_{key}.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def collect_paths(args):
    paths = list(args.files)
    if args.dir:
        paths.extend(sorted(glob.glob(os.path.join(args.dir, args.glob))))
    # If nothing was specified, default to data files sitting next to this script.
    if not paths:
        paths = sorted(glob.glob(os.path.join(_HERE, args.glob)))
    return paths


def main():
    parser = argparse.ArgumentParser(description="Cluster one or more point files.")
    parser.add_argument("files", nargs="*", help="point files to cluster")
    parser.add_argument("--dir", help="directory to pull files from")
    parser.add_argument("--glob", default="*.txt",
                        help="glob pattern when using --dir (default: *.txt)")
    parser.add_argument("--visualize", action="store_true",
                        help="save a scatter plot per file (2D/3D)")
    parser.add_argument("--outdir", default="plots",
                        help="where to save plots (default: plots)")
    parser.add_argument("--group-by", metavar="KEY",
                        help="cluster each 'KEY=value' tag bucket separately")
    parser.add_argument("--include-untagged", action="store_true",
                        help="with --group-by, also cluster points lacking the key")
    args = parser.parse_args()

    paths = collect_paths(args)
    if not paths:
        parser.error(f"no input files found (looked in {_HERE} for {args.glob})")

    print(f"Testing {len(paths)} file(s)\n")

    n_ok = n_fail = 0
    for path in paths:
        status, info = process_file(path)

        if args.group_by and status == "ok":
            print_grouped_result(path, info, args.group_by, args.include_untagged)
            if args.visualize:
                out = visualize_grouped(path, info, args.group_by,
                                        args.include_untagged, args.outdir)
                if out:
                    print(f"        plot saved -> {out}")
            n_ok += 1
            continue

        print_result(path, status, info)
        if status == "ok":
            n_ok += 1
            if args.visualize:
                out = visualize(path, info, args.outdir)
                if out:
                    print(f"        plot saved -> {out}")
        elif status == "error":
            n_fail += 1

    print(f"\nSummary: {n_ok} ok, {n_fail} failed, {len(paths)} total")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()

