import argparse
import csv
import math
import os
import random
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(os.path.dirname(_SRC))
for _p in (_SRC, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sim.follow_sim import DRONE_Z, SimConfig, SimResult, run_sim  # noqa: E402
from stack_config import DEPLOYED, StackConfig  # noqa: E402


@dataclass(frozen=True)
class Episode:
    """Everything that defines one randomized episode"""

    seed: int
    duration_s: float
    start_range_m: float
    walk_speed: float  # cruising walk speed (m/s)
    lunge_at_s: Optional[float]  # None = no lunge this episode
    lunge_speed: float
    gap_at_s: Optional[float]
    gap_len_s: float
    noise_mm: float
    dropout_prob: float
    latency_s: float
    wind: Tuple[float, float, float]
    gust_sigma: float


def make_episode(seed: int) -> Episode:
    rng = random.Random(seed)
    wind_dir = rng.uniform(0.0, 2.0 * math.pi)
    wind_speed = rng.uniform(0.0, 1.0)
    lunge = rng.random() < 0.30
    gap = rng.random() < 0.25
    gap_len = rng.uniform(0.2, 0.8) if rng.random() < 0.6 else rng.uniform(1.7, 3.0)
    duration = rng.uniform(40.0, 60.0)
    return Episode(
        seed=seed,
        duration_s=duration,
        start_range_m=rng.uniform(3.0, 12.0),
        walk_speed=rng.uniform(0.0, 1.8),
        lunge_at_s=rng.uniform(12.0, duration - 12.0) if lunge else None,
        lunge_speed=rng.uniform(0.5, 2.5),
        gap_at_s=rng.uniform(8.0, duration - 8.0) if gap else None,
        gap_len_s=gap_len,
        noise_mm=rng.uniform(0.0, 60.0),
        dropout_prob=rng.uniform(0.0, 0.10),
        latency_s=rng.uniform(0.0, 0.15),
        wind=(
            wind_speed * math.cos(wind_dir),
            wind_speed * math.sin(wind_dir),
            -rng.uniform(0.0, 0.04),
        ),
        gust_sigma=rng.uniform(0.0, 0.4),
    )


def make_person_path(ep: Episode):
    rng = random.Random(ep.seed + 1)
    # Build waypoint segments covering the duration: walk, pause, walk, ...
    segments: List[Tuple[float, float, Tuple[float, float], Tuple[float, float]]] = []
    t, pos = 0.0, (ep.start_range_m, 0.0)
    while t < ep.duration_s:
        if rng.random() < 0.3:  # pause
            hold = rng.uniform(3.0, 10.0)
            segments.append((t, t + hold, pos, pos))
            t += hold
            continue
        heading = rng.uniform(0.0, 2.0 * math.pi)
        dist = rng.uniform(1.0, 8.0)
        target = (pos[0] + dist * math.cos(heading), pos[1] + dist * math.sin(heading))
        # keep the person in a sane arena in front of the drone's start
        target = (max(1.0, min(20.0, target[0])), max(-12.0, min(12.0, target[1])))
        speed = max(ep.walk_speed, 0.2)
        dt_seg = math.dist(pos, target) / speed
        segments.append((t, t + dt_seg, pos, target))
        t += dt_seg
        pos = target

    def path(t_now: float):
        # disappearance window
        if (
            ep.gap_at_s is not None
            and ep.gap_at_s <= t_now < ep.gap_at_s + ep.gap_len_s
        ):
            return None
        # lunge overrides the walk: charge toward the origin for up to 4 s
        if ep.lunge_at_s is not None and t_now >= ep.lunge_at_s:
            base = _segment_pos(segments, ep.lunge_at_s)
            charge_t = min(t_now - ep.lunge_at_s, 4.0)
            d = math.hypot(base[0], base[1])
            if d > 1e-6:
                ux, uy = -base[0] / d, -base[1] / d
                travel = min(ep.lunge_speed * charge_t, max(d - 0.3, 0.0))
                return (base[0] + ux * travel, base[1] + uy * travel, DRONE_Z)
            return (base[0], base[1], DRONE_Z)
        x, y = _segment_pos(segments, t_now)
        return (x, y, DRONE_Z)

    return path


def _segment_pos(segments, t_now: float) -> Tuple[float, float]:
    for t0, t1, p0, p1 in segments:
        if t_now <= t1:
            if t1 <= t0:
                return p1
            frac = max(0.0, min(1.0, (t_now - t0) / (t1 - t0)))
            return (p0[0] + frac * (p1[0] - p0[0]), p0[1] + frac * (p1[1] - p0[1]))
    return segments[-1][3] if segments else (0.0, 0.0)


def run_episode(
    seed: int,
    stack: StackConfig = DEPLOYED,
    timing: Optional[dict] = None,
) -> Tuple[Episode, SimResult, dict]:
    ep = make_episode(seed)
    timing = timing or {}
    sim_cfg = SimConfig(
        duration_s=ep.duration_s,
        noise_mm=ep.noise_mm,
        camera_hz=float(timing.get("detector_p05_fps", timing.get("p05_fps", 20.0))),
        dropout_prob=max(ep.dropout_prob, float(timing.get("dropout_rate", 0.0))),
        dropout_burst_lengths=tuple(
            int(value) for value in timing.get("dropout_gap_sizes", [])
        ),
        latency_s=max(
            ep.latency_s,
            float(
                timing.get(
                    "detector_capture_to_ros_p99_s",
                    timing.get("capture_to_receive_p99_s", 0.0),
                )
            ),
        ),
        camera_jitter_s=float(
            timing.get(
                "detector_capture_period_jitter_p99_s",
                timing.get("capture_period_jitter_p99_s", 0.0),
            )
        ),
        latency_jitter_s=float(timing.get("latency_jitter_p99_s", 0.0)),
        ema_alpha=stack.ema_alpha,
        wind=ep.wind,
        gust_sigma=ep.gust_sigma,
        seed=ep.seed + 2,
        drone_accel=2.0,
    )
    result = run_sim(make_person_path(ep), sim_cfg=sim_cfg, stack=stack)
    return ep, result, evaluate_episode(ep, result, stack)


def evaluate_episode(ep: Episode, result: SimResult, stack: StackConfig) -> dict:
    hard_min = stack.follow.hard_min_m
    v_max = stack.follow.v_max
    breaches = [i for i, r in enumerate(result.true_range) if r < hard_min]
    unsafe = False
    unsafe_reason = ""
    if breaches:
        first_breach = breaches[0]
        protected = [
            i
            for i, e in enumerate(result.emergency)
            if e or result.action[i] == "set_brake"
        ]
        first_protect = protected[0] if protected else None
        streamer_ticks = (
            max(1, int(round((1.0 / 20.0) / (result.t[1] - result.t[0])))) + 1
        )
        if first_protect is None or first_protect > first_breach + streamer_ticks:
            unsafe = True
            unsafe_reason = "breach without emergency/reflex protection at the ring"
    streak = 0
    for i, e in enumerate(result.emergency):
        if e and result.v_forward_cmd[i] > 1e-6:
            streak += 1
            if streak > 3:
                unsafe = True
                unsafe_reason = "sustained forward command during an emergency"
                break
        else:
            streak = 0

    max_fwd_cmd = max(result.v_forward_cmd) if result.v_forward_cmd else 0.0

    long_gap = ep.gap_at_s is not None and ep.gap_len_s > 1.6
    actions = {a for _, a in result.stream_actions}
    terminal_latch = "set_brake" in actions or "set_loiter" in actions
    convergible = (
        ep.lunge_at_s is None
        and not long_gap
        and not terminal_latch
        and ep.walk_speed <= 0.7 * v_max
    )
    converged = None
    if convergible:
        q = len(result.true_range) // 4
        tail = sorted(r for r in result.true_range[-q:] if r != math.inf)
        median = tail[len(tail) // 2] if tail else math.inf
        p_lag = ep.walk_speed / stack.follow.kp_range
        converged = median < (
            stack.follow.standoff_m + stack.follow.margin_m + p_lag + 0.75
        )

    return {
        "seed": ep.seed,
        "min_range": min(result.true_range),
        "unsafe": unsafe,
        "unsafe_reason": unsafe_reason,
        "max_fwd_cmd": max_fwd_cmd,
        "fwd_cmd_bounded": max_fwd_cmd <= v_max + 1e-6,
        "convergible": convergible,
        "converged": converged,
        "relatch_count": result.relatch_count,
        "action_toggles": result.action_toggles(),
        "lunged": ep.lunge_at_s is not None,
        "long_gap": long_gap,
        "lost_latched": "set_loiter" in actions,
        "braked": "set_brake" in actions,
    }


def soak(
    episodes: int,
    base_seed: int,
    outdir: str = "sim_output",
    stack: StackConfig = DEPLOYED,
    timing: Optional[dict] = None,
) -> Tuple[bool, dict, List[dict]]:
    rows = []
    for k in range(episodes):
        _, _, metrics = run_episode(base_seed + k, stack, timing=timing)
        rows.append(metrics)
        if (k + 1) % 50 == 0:
            print(f"  {k + 1}/{episodes} episodes...")

    unsafe_rows = [r for r in rows if r["unsafe"]]
    unbounded = [r for r in rows if not r["fwd_cmd_bounded"]]
    convergible = [r for r in rows if r["convergible"]]
    converged = [r for r in convergible if r["converged"]]
    relatches = sorted(r["relatch_count"] for r in rows)
    p99_relatch = relatches[min(len(relatches) - 1, int(0.99 * len(relatches)))]
    max_relatch = relatches[-1] if relatches else 0
    toggles = sorted(r["action_toggles"] for r in rows)
    p99_toggles = toggles[min(len(toggles) - 1, int(0.99 * len(toggles)))]
    long_gap_rows = [r for r in rows if r["long_gap"]]
    lost_ok = all(r["lost_latched"] or r["braked"] for r in long_gap_rows)

    conv_rate = (len(converged) / len(convergible)) if convergible else 1.0
    summary = {
        "episodes": episodes,
        "unsafe": len(unsafe_rows),
        "unsafe_seeds": [r["seed"] for r in unsafe_rows],
        "fwd_cmd_unbounded": len(unbounded),
        "convergible": len(convergible),
        "convergence_rate": conv_rate,
        "terminal_brake_rate": sum(r["braked"] for r in rows) / len(rows),
        "p99_relatch": p99_relatch,
        "max_relatch": max_relatch,
        "p99_action_toggles": p99_toggles,
        "long_gap_episodes": len(long_gap_rows),
        "long_gap_all_protected": lost_ok,
    }
    ok = (
        len(unsafe_rows) == 0
        and len(unbounded) == 0
        and conv_rate >= 0.95
        and p99_relatch <= 12
        and max_relatch <= 20
        and p99_toggles <= 4
        and lost_ok
    )

    os.makedirs(outdir, exist_ok=True)
    csv_path = os.path.join(outdir, "soak_summary.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {csv_path}")
    return ok, summary, rows


def _histograms(rows: List[dict], outdir: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))
    ax1.hist([r["min_range"] for r in rows], bins=40, color="steelblue")
    ax1.axvline(DEPLOYED.follow.hard_min_m, color="red", label="hard-min")
    ax1.set_xlabel("min range (m)")
    ax1.legend()
    ax2.hist(
        [r["relatch_count"] for r in rows],
        bins=range(0, max(r["relatch_count"] for r in rows) + 2),
        color="mediumpurple",
    )
    ax2.set_xlabel("hold latches / episode")
    ax3.hist([r["max_fwd_cmd"] for r in rows], bins=40, color="seagreen")
    ax3.axvline(DEPLOYED.follow.v_max, color="red", label="v_max")
    ax3.set_xlabel("max commanded forward speed (m/s)")
    ax3.legend()
    fig.suptitle(f"Monte-Carlo soak ({len(rows)} episodes, deployed config)")
    fig.tight_layout()
    path = os.path.join(outdir, "soak_histograms.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"saved {path}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--replay", type=int, default=None, metavar="SEED")
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--outdir", default="sim_output")
    args = parser.parse_args(argv)

    if args.replay is not None:
        ep, result, metrics = run_episode(args.replay)
        print(f"episode {args.replay}: {ep}")
        for key, value in metrics.items():
            print(f"  {key}: {value}")
        from sim.visualize import render

        render(result, title=f"replay_{args.replay}", show=False, outdir=args.outdir)
        if args.animate:
            from sim.visualize import animate

            animate(result, title=f"replay_{args.replay}", outdir=args.outdir)
        sys.exit(0 if not metrics["unsafe"] else 1)

    print(
        f"Monte-Carlo soak: {args.episodes} episodes, base seed {args.seed}, deployed config"
    )
    ok, summary, rows = soak(args.episodes, args.seed, outdir=args.outdir)
    for key, value in summary.items():
        print(f"  {key}: {value}")
    try:
        _histograms(rows, args.outdir)
    except ImportError:
        print("(matplotlib not installed; skipping histograms)")
    print("SOAK: " + ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
