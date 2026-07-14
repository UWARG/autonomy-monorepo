"""Matplotlib rendering for the follow simulation (opt-in; needs the `viz` extra).

Kept out of follow_sim's import path so the Gate 2 assertions run headless without
matplotlib.

``render``  -- a 6-panel diagnostic figure saved under gnc/sim_output/.
``animate`` -- a top-down animated replay (MP4 via ffmpeg, GIF fallback) showing
               the drone, the person, the standoff/hard-min rings, the commanded
               velocity arrow, and a state banner (action / hold / emergency).
"""

import math
import os

ACTION_ORDER = [
    "release",
    "set_loiter",
    "set_brake",
    "stream_zero",
    "hold_position",
    "stream_velocity",
]


def _action_series(result):
    """stream_actions -> step series aligned to result.t (index of ACTION_ORDER)."""
    levels = []
    j = 0
    current = 0
    for t in result.t:
        while j < len(result.stream_actions) and result.stream_actions[j][0] <= t:
            name = result.stream_actions[j][1]
            current = ACTION_ORDER.index(name) if name in ACTION_ORDER else 0
            j += 1
        levels.append(current)
    return levels


def _shade_spans(ax, t, flags, color, alpha, label=None):
    """Shade contiguous True spans of ``flags`` on ``ax``."""
    start = None
    labeled = False
    for i, f in enumerate(flags):
        if f and start is None:
            start = t[i]
        elif not f and start is not None:
            ax.axvspan(start, t[i], color=color, alpha=alpha,
                       label=label if not labeled else None)
            labeled = True
            start = None
    if start is not None:
        ax.axvspan(start, t[-1], color=color, alpha=alpha,
                   label=label if not labeled else None)


def render(result, title: str = "follow", show: bool = True, outdir: str = "sim_output"):
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(2, 3, figsize=(17, 9))
    (ax_xy, ax_r, ax_v), (ax_z, ax_a, ax_p) = axes
    fig.suptitle(f"Follow sim: {title}")

    # --- top-down trajectory ---
    ax_xy.plot(result.drone_x, result.drone_y, "b-", label="drone path", lw=2)
    px = [x for x in result.person_x if x == x]  # drop NaN
    py = [result.person_y[i] for i, x in enumerate(result.person_x) if x == x]
    ax_xy.plot(px, py, "g-", label="person path", lw=2)
    if px:
        cx, cy = px[-1], py[-1]
        ax_xy.add_patch(Circle((cx, cy), result.standoff_m, color="orange", fill=False,
                               ls="--", label="standoff"))
        ax_xy.add_patch(Circle((cx, cy), result.hard_min_m, color="red", fill=False,
                               label="hard-min"))
        ax_xy.plot([cx], [cy], "g*", ms=14)
    for _, hx, hy, _hz in result.hold_points:
        ax_xy.plot([hx], [hy], "m^", ms=8)
    ax_xy.plot([result.drone_x[0]], [result.drone_y[0]], "bo", label="drone start")
    ax_xy.set_aspect("equal", "box")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.legend(loc="best", fontsize=8)
    ax_xy.grid(True)

    # --- range vs time ---
    ax_r.plot(result.t, result.true_range, "b-", label="true range")
    ax_r.axhline(result.standoff_m, color="orange", ls="--", label="standoff")
    ax_r.axhline(result.hard_min_m, color="red", label="hard-min")
    _shade_spans(ax_r, result.t, result.emergency, "red", 0.12, "emergency")
    finite = [r for r in result.true_range if r != math.inf]
    ax_r.set_ylim(0, max(7, (max(finite) if finite else 6) + 1))
    ax_r.set_xlabel("t (s)")
    ax_r.set_ylabel("range (m)")
    ax_r.legend(loc="best", fontsize=8)
    ax_r.grid(True)

    # --- commanded forward speed vs braking cap ---
    ax_v.plot(result.t, result.v_forward_cmd, "b-", label="v_forward cmd")
    ax_v.plot(result.t, result.v_brake_cap, "r--", label="v_brake_cap")
    ax_v.axhline(0, color="k", lw=0.5)
    ax_v.set_xlabel("t (s)")
    ax_v.set_ylabel("speed (m/s)")
    ax_v.legend(loc="best", fontsize=8)
    ax_v.grid(True)

    # --- altitude + hold shading ---
    ax_z.plot(result.t, result.drone_z, "b-", label="altitude")
    for th, _hx, _hy, hz in result.hold_points:
        ax_z.plot([th], [hz], "m^", ms=8)
    _shade_spans(ax_z, result.t, result.hold_active, "magenta", 0.10, "hold active")
    ax_z.set_xlabel("t (s)")
    ax_z.set_ylabel("z (m)")
    ax_z.legend(loc="best", fontsize=8)
    ax_z.grid(True)

    # --- ladder action timeline ---
    ax_a.step(result.t, _action_series(result), where="post", lw=1.2)
    ax_a.set_yticks(range(len(ACTION_ORDER)))
    ax_a.set_yticklabels(ACTION_ORDER, fontsize=7)
    ax_a.set_xlabel("t (s)")
    ax_a.set_title(
        f"ladder action (latches={result.relatch_count}, "
        f"vel/zero toggles={result.action_toggles()})",
        fontsize=9,
    )
    ax_a.grid(True)

    # --- vertical channel: command vs actual + pitch ---
    ax_p.plot(result.t, result.v_down_cmd, "b-", label="v_down cmd", lw=1)
    ax_p.plot(result.t, result.vz, "c-", label="vz (world up)", lw=1)
    ax_p.plot(result.t, [math.degrees(p) / 10.0 for p in result.pitch], "k--",
              label="pitch (deg/10)", lw=1)
    ax_p.axhline(0, color="k", lw=0.5)
    ax_p.set_xlabel("t (s)")
    ax_p.set_ylabel("m/s")
    ax_p.legend(loc="best", fontsize=8)
    ax_p.grid(True)

    fig.tight_layout()
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, f"{title}.png")
    fig.savefig(path, dpi=110)
    print(f"saved {path}")
    if show:
        plt.show()
    plt.close(fig)
    return path


def animate(
    result,
    title: str = "follow",
    outdir: str = "sim_output",
    fps: int = 25,
    speedup: float = 2.0,
    show: bool = False,
):
    """Top-down animated replay -> MP4 (ffmpeg) or GIF (Pillow fallback).

    ``speedup`` plays the episode faster than real time (2x default) to keep
    files small; one animation frame per (speedup / fps) seconds of sim time.
    """
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Circle

    dt = result.t[1] - result.t[0] if len(result.t) > 1 else 0.02
    stride = max(1, int(round(speedup / (fps * dt))))
    frames = range(0, len(result.t), stride)

    fig, ax = plt.subplots(figsize=(8, 8))
    xs = result.drone_x + [x for x in result.person_x if x == x]
    ys = result.drone_y + [y for y in result.person_y if y == y]
    pad = 2.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal", "box")
    ax.grid(True)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    (drone_trail,) = ax.plot([], [], "b-", lw=1, alpha=0.6)
    (drone_dot,) = ax.plot([], [], "bo", ms=9, label="drone")
    (person_dot,) = ax.plot([], [], "g*", ms=14, label="person")
    standoff_ring = Circle((0, 0), result.standoff_m, color="orange", fill=False, ls="--")
    hard_ring = Circle((0, 0), result.hard_min_m, color="red", fill=False)
    ax.add_patch(standoff_ring)
    ax.add_patch(hard_ring)
    vel_arrow = ax.annotate(
        "", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="blue", lw=2)
    )
    banner = ax.set_title("")
    ax.legend(loc="upper right", fontsize=8)

    def update(i):
        dx, dy = result.drone_x[i], result.drone_y[i]
        px, py = result.person_x[i], result.person_y[i]
        trail0 = max(0, i - int(8.0 / dt))
        drone_trail.set_data(result.drone_x[trail0:i + 1], result.drone_y[trail0:i + 1])
        drone_dot.set_data([dx], [dy])
        visible = px == px
        person_dot.set_visible(visible)
        if visible:
            person_dot.set_data([px], [py])
            standoff_ring.center = (px, py)
            hard_ring.center = (px, py)
        # commanded velocity arrow (forward command along the drone->person bearing
        # is a fair visual proxy in this top-down view)
        vf = result.v_forward_cmd[i]
        if visible and math.isfinite(result.true_range[i]) and result.true_range[i] > 1e-6:
            ux = (px - dx) / result.true_range[i]
            uy = (py - dy) / result.true_range[i]
        else:
            ux = uy = 0.0
        vel_arrow.xy = (dx + ux * vf * 2.0, dy + uy * vf * 2.0)
        vel_arrow.set_position((dx, dy))
        state = result.action[i]
        if result.hold_active[i]:
            state += " | HOLD"
        if result.emergency[i]:
            state += " | EMERGENCY"
        banner.set_text(f"{title}   t={result.t[i]:5.1f} s   {state}")
        drone_dot.set_color("red" if result.emergency[i] else
                            ("magenta" if result.hold_active[i] else "blue"))
        return drone_trail, drone_dot, person_dot

    anim = animation.FuncAnimation(fig, update, frames=frames, interval=1000 / fps)
    os.makedirs(outdir, exist_ok=True)
    base = os.path.join(outdir, title)
    try:
        path = base + ".mp4"
        anim.save(path, writer=animation.FFMpegWriter(fps=fps))
    except (FileNotFoundError, RuntimeError, ValueError):
        path = base + ".gif"
        anim.save(path, writer=animation.PillowWriter(fps=fps))
    print(f"saved {path}")
    if show:
        plt.show()
    plt.close(fig)
    return path
