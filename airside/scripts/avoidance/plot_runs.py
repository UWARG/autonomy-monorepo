#!/usr/bin/env python3
"""Render top-down trajectory figures from SITL logs."""

from __future__ import annotations

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "logs")
OUT = os.path.join(HERE, "figures")
os.makedirs(OUT, exist_ok=True)

WALL_N, WALL_HALF, MARGIN, GOAL_N = 20, 6, 3, 40

INK = "#1b2430"
MUTED = "#5f6f88"
GRID = "#d9e0ea"
CYAN = "#1f9ecf"
PASS = "#1f9d57"
FAIL = "#e0432f"
HAZARD = "#e0432f"

RUNS = {
    "wall_guided": {
        "title": "Plain GUIDED goto",
        "verdict": "FAIL",
        "min": 0.38,
        "note": "position target bypasses avoidance",
        "goal": True,
    },
    "wall_guided_wpnav": {
        "title": "GUIDED + GUID_OPTIONS=64",
        "verdict": "PASS",
        "min": 3.84,
        "note": "routed via WPNav → detours",
        "goal": True,
    },
    "wall_guided_vel": {
        "title": "GUIDED velocity stream",
        "verdict": "PASS",
        "min": 0.77,
        "note": "simple avoidance stops it short",
        "goal": False,
    },
    "wall_auto": {
        "title": "AUTO mission",
        "verdict": "PASS",
        "min": 3.79,
        "note": "BendyRuler detours, continues",
        "goal": True,
    },
    "clear_guided": {
        "title": "Baseline · no obstacle",
        "verdict": "PASS",
        "min": None,
        "note": "straight to goal",
        "goal": True,
    },
}


def load(scen):
    pts = []
    with open(os.path.join(LOGS, f"{scen}.jsonl")) as f:
        for line in f:
            d = json.loads(line)
            pts.append((d["east_m"], d["north_m"]))
    return pts


def draw(ax, scen, meta, big=False, show_note=True):
    xs = [p[0] for p in load(scen)]
    ys = [p[1] for p in load(scen)]
    col = PASS if meta["verdict"] == "PASS" else FAIL
    haswall = scen != "clear_guided"

    ax.set_facecolor("white")
    ax.set_xlim(-15, 15)
    ax.set_ylim(-3, 43)
    ax.set_aspect("equal")
    ax.grid(True, color=GRID, lw=0.8, zorder=0)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)

    if haswall:
        ax.add_patch(
            Rectangle(
                (-WALL_HALF - MARGIN, WALL_N - MARGIN),
                2 * (WALL_HALF + MARGIN),
                2 * MARGIN,
                facecolor=HAZARD,
                alpha=0.07,
                edgecolor=HAZARD,
                ls="--",
                lw=1,
                zorder=1,
            )
        )
        ax.plot(
            [-WALL_HALF, WALL_HALF],
            [WALL_N, WALL_N],
            color=HAZARD,
            lw=5,
            solid_capstyle="round",
            zorder=4,
        )

    ax.plot(xs, ys, color=col, lw=2.4, zorder=5, solid_capstyle="round")
    ax.scatter([0], [0], s=42, color=INK, zorder=6)
    ax.scatter(
        [xs[-1]],
        [ys[-1]],
        s=70,
        color=col,
        edgecolor="white",
        lw=1.4,
        zorder=7,
        marker="^",
    )
    if meta["goal"]:
        ax.scatter(
            [0], [GOAL_N], s=90, facecolor="none", edgecolor=MUTED, lw=1.6, zorder=6
        )
        ax.scatter([0], [GOAL_N], s=12, color=MUTED, zorder=6)

    mind = "" if meta["min"] is None else f"  ·  min {meta['min']:.2f} m"
    tsize = 12 if big else 10.5
    caption = (
        f"{meta['verdict']}{mind} — {meta['note']}"
        if show_note
        else f"{meta['verdict']}{mind}"
    )
    ax.set_title(
        f"{meta['title']}",
        color=INK,
        fontsize=tsize,
        fontweight="bold",
        pad=22 if big else 20,
    )
    ax.text(
        0.5,
        1.012,
        caption,
        transform=ax.transAxes,
        ha="center",
        va="bottom",
        color=col,
        fontsize=9 if big else 8.5,
        fontweight="bold",
    )


def hero():
    fig, axes = plt.subplots(1, 2, figsize=(10, 6.2))
    fig.suptitle(
        "Same wall, different command path",
        fontsize=15,
        fontweight="bold",
        color=INK,
        y=0.98,
    )
    draw(axes[0], "wall_guided", RUNS["wall_guided"], big=True)
    draw(axes[1], "wall_auto", RUNS["wall_auto"], big=True)
    axes[0].set_ylabel("north (m)", color=MUTED, fontsize=9)
    for ax in axes:
        ax.set_xlabel("east (m)", color=MUTED, fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    p = os.path.join(OUT, "avoidance_hero.png")
    fig.savefig(p, dpi=130, facecolor="white", bbox_inches="tight")
    print("wrote", p)


def summary():
    order = [
        "clear_guided",
        "wall_guided",
        "wall_guided_wpnav",
        "wall_guided_vel",
        "wall_auto",
    ]
    fig, axes = plt.subplots(1, 5, figsize=(18, 4.6))
    fig.suptitle(
        "ArduPilot avoidance — five SITL runs against a 12 m wall",
        fontsize=14,
        fontweight="bold",
        color=INK,
        y=1.05,
    )
    for ax, scen in zip(axes, order):
        draw(ax, scen, RUNS[scen], show_note=False)
    axes[0].set_ylabel("north (m)", color=MUTED, fontsize=9)
    fig.tight_layout()
    p = os.path.join(OUT, "avoidance_summary.png")
    fig.savefig(p, dpi=120, facecolor="white", bbox_inches="tight")
    print("wrote", p)


if __name__ == "__main__":
    hero()
    summary()
