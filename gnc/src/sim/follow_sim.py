import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(os.path.dirname(_SRC))
for _p in (_SRC, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from flight_modes import ControlAction, FlightInputs, decide  # noqa: E402
from follow_controller import FollowConfig, compute_setpoint  # noqa: E402
from hold_policy import HoldAction, HoldConfig, HoldPolicy  # noqa: E402
from mavros_setpoint import slew  # noqa: E402
from range_rate import ReflexConfig, ReflexMonitor  # noqa: E402
from safety import EStopAction, SafetyConfig, SafetyMonitor  # noqa: E402
from stack_config import StackConfig  # noqa: E402
from utils import Rotation, Vector3D  # noqa: E402

# A person path maps time (s) -> world (x, y, z) position, None when the person
# is not detectable
PersonPath = Callable[[float], Optional[Tuple[float, float, float]]]

DRONE_Z = 1.5  # drone start altitude; person torso defaults to this height
G = 9.81


@dataclass
class SimConfig:
    dt: float = 0.02 
    duration_s: float = 40.0
    drone_accel: float = 1.0  
    vert_accel: float = 2.0 
    yaw_accel: float = 4.0  # rad/s^2
    camera_hz: float = 20.0  
    noise_mm: float = 0.0 
    dropout_prob: float = 0.0
    latency_s: float = 0.0 
    ema_alpha: Optional[float] = None 
    recede_speed: float = 1.0 
    sign_error: bool = False
    seed: int = 0
    start_x: float = 6.0  
    # --- disturbances ---
    wind: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gust_sigma: float = 0.0 
    gust_tau_s: float = 2.0
    pitch_coupling: bool = False 
    pitch_tau_s: float = 0.3
    wind_rejection: float = 0.7
    # --- estimator / ladder inputs ---
    ekf_ok: bool = True
    legacy_stale_semantics: bool = False
    kp_pos: float = 1.0
    hold_speed_limit: float = 1.0


@dataclass
class SimResult:
    t: List[float] = field(default_factory=list)
    drone_x: List[float] = field(default_factory=list)
    drone_y: List[float] = field(default_factory=list)
    drone_z: List[float] = field(default_factory=list)
    person_x: List[float] = field(default_factory=list)
    person_y: List[float] = field(default_factory=list)
    true_range: List[float] = field(default_factory=list)
    v_forward_cmd: List[float] = field(default_factory=list)
    v_down_cmd: List[float] = field(default_factory=list)
    vz: List[float] = field(default_factory=list)
    pitch: List[float] = field(default_factory=list)
    v_brake_cap: List[float] = field(default_factory=list)
    emergency: List[bool] = field(default_factory=list)
    hold_active: List[bool] = field(default_factory=list)
    action: List[str] = field(default_factory=list)  # ladder action, per physics tick
    stream_actions: List[Tuple[float, str]] = field(default_factory=list)  # per streamer tick
    hold_points: List[Tuple[float, float, float, float]] = field(default_factory=list)  # t,x,y,z
    relatch_count: int = 0
    standoff_m: float = 2.5
    hard_min_m: float = 1.5

    @property
    def min_true_range(self) -> float:
        return min(self.true_range) if self.true_range else math.inf

    @property
    def final_range(self) -> float:
        return self.true_range[-1] if self.true_range else math.inf

    @property
    def min_altitude(self) -> float:
        return min(self.drone_z) if self.drone_z else math.inf

    def action_toggles(self, a: str = "stream_velocity", b: str = "stream_zero") -> int:
        """Count a<->b transitions in the streamer-tick action sequence."""
        toggles = 0
        prev = None
        for _, act in self.stream_actions:
            if act not in (a, b):
                prev = None
                continue
            if prev is not None and act != prev:
                toggles += 1
            prev = act
        return toggles

    @property
    def hold_point_drift(self) -> float:
        """Distance between the first and last latched hold points"""
        if len(self.hold_points) < 2:
            return 0.0
        _, x0, y0, z0 = self.hold_points[0]
        _, x1, y1, z1 = self.hold_points[-1]
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)


def _camera_reading(rel_fwd, rel_right, rel_down, cam_pitch, rng, noise_mm):
    aligned = Rotation(Vector3D(0.0, 1.0, 0.0), cam_pitch).rotate_vector3d(
        Vector3D(rel_fwd, rel_right, rel_down)
    )
    cam_z_mm = aligned.x * 1000.0
    cam_x_mm = aligned.y * 1000.0
    cam_y_mm = aligned.z * 1000.0
    if noise_mm > 0.0:
        cam_x_mm += rng.gauss(0.0, noise_mm)
        cam_y_mm += rng.gauss(0.0, noise_mm)
        cam_z_mm += rng.gauss(0.0, noise_mm)
    return cam_x_mm, cam_y_mm, cam_z_mm


def run_sim(
    person_path: PersonPath,
    follow_cfg: FollowConfig = FollowConfig(),
    sim_cfg: SimConfig = SimConfig(),
    stack: Optional[StackConfig] = None,
) -> SimResult:
    if stack is not None:
        follow_cfg = stack.follow
        safety_cfg = stack.safety
        hold_cfg = stack.hold
        reflex_cfg = stack.reflex
        tree_hz = stack.tree_hz
        stream_hz = stack.stream_hz
        command_stale_s = stack.command_stale_s
        target_freshness_s = stack.target_freshness_s
        recede_speed = stack.recede_speed
    else:
        safety_cfg = SafetyConfig(hard_min_m=follow_cfg.hard_min_m, a_brake=follow_cfg.a_brake)
        hold_cfg = HoldConfig()
        reflex_cfg = ReflexConfig(hard_min_m=follow_cfg.hard_min_m, a_brake=follow_cfg.a_brake)
        tree_hz = 2.0
        stream_hz = 20.0
        command_stale_s = StackConfig().command_stale_s
        target_freshness_s = StackConfig().target_freshness_s
        recede_speed = sim_cfg.recede_speed

    rng = random.Random(sim_cfg.seed)
    monitor = SafetyMonitor(safety_cfg)
    hold = HoldPolicy(hold_cfg)
    reflex = ReflexMonitor(reflex_cfg)

    # Drone state: position, velocity, heading (clockwise+ from +X), yaw rate, pitch.
    dx, dy, dz = 0.0, 0.0, DRONE_Z
    vx, vy, vz = 0.0, 0.0, 0.0
    psi, yaw_rate = 0.0, 0.0
    theta = 0.0  # body pitch (rad); + = nose down (accelerating forward)
    gust = [0.0, 0.0, 0.0]

    # Camera/detection pipeline state.
    ema = None
    pending: List[Tuple[float, Tuple[float, float, float], float]] = []  # (t_avail, cam_mm, range)
    latest_cam: Optional[Tuple[float, float, float]] = None  # mm
    latest_range: Optional[float] = None
    latest_rx_t = -math.inf

    # Tree outputs.
    cmd: Optional[Tuple[float, float, float, float]] = None
    cmd_stamp = -math.inf
    estop_emergency = False
    estop_recede = False
    estop_hold = False
    last_verdict_emergency = False

    # Streamer / flight-mode state.
    mode = "GUIDED"
    hold_point: Optional[Tuple[float, float, float, float]] = None  # x, y, z, psi
    fc_hold_point: Optional[Tuple[float, float, float]] = None
    control: Tuple[str, Tuple[float, float, float, float]] = ("velocity", (0.0, 0.0, 0.0, 0.0))
    last_action_name = "stream_velocity"
    streamed = (0.0, 0.0, 0.0) 
    dv_max = (stack.cmd_slew_mps2 if stack is not None else StackConfig().cmd_slew_mps2) / stream_hz

    result = SimResult(standoff_m=follow_cfg.standoff_m, hard_min_m=follow_cfg.hard_min_m)

    steps = int(sim_cfg.duration_s / sim_cfg.dt)
    cam_period = 1.0 / sim_cfg.camera_hz
    tree_period = 1.0 / tree_hz
    stream_period = 1.0 / stream_hz
    next_cam_t = 0.0
    next_tree_t = 0.0
    next_stream_t = 0.0

    for i in range(steps):
        t = i * sim_cfg.dt
        person = person_path(t)

        true_range = math.inf
        rel = None
        if person is not None:
            px, py, pz = person
            ex, ey, ez = px - dx, py - dy, pz - dz
            # world -> body (forward, right, down). psi clockwise+ from +X.
            cos_p, sin_p = math.cos(psi), math.sin(psi)
            rel = (
                ex * cos_p - ey * sin_p,  # forward
                -ex * sin_p - ey * cos_p,  # right
                -ez,  # down
            )
            true_range = math.sqrt(ex * ex + ey * ey + ez * ez)
        else:
            px = py = math.nan

        # --- camera at camera_hz: capture -> (optional latency) -> deliver ---
        if t >= next_cam_t:
            next_cam_t += cam_period
            if rel is not None and rng.random() >= sim_cfg.dropout_prob:
                cam_pitch = follow_cfg.mount_pitch_rad + (
                    theta if sim_cfg.pitch_coupling else 0.0
                )
                cam = _camera_reading(
                    rel[0], rel[1], rel[2], cam_pitch, rng, sim_cfg.noise_mm
                )
                if sim_cfg.ema_alpha is not None:
                    a = sim_cfg.ema_alpha
                    ema = (
                        cam
                        if ema is None
                        else tuple(a * c + (1 - a) * e for c, e in zip(cam, ema))
                    )
                    cam = ema
                rng_m = math.sqrt(cam[0] ** 2 + cam[1] ** 2 + cam[2] ** 2) / 1000.0
                pending.append((t + sim_cfg.latency_s, cam, rng_m))
            elif rel is None:
                ema = None
        while pending and pending[0][0] <= t:
            _, latest_cam, latest_range = pending.pop(0)
            latest_rx_t = t

        target_fresh = (t - latest_rx_t) <= target_freshness_s

        # --- behavior tree at tree_hz: SafetyMonitor arbitration + follow command ---
        if t >= next_tree_t:
            next_tree_t += tree_period
            measured = latest_range if target_fresh else None
            verdict = monitor.evaluate(measured, tree_period)
            estop_emergency = verdict.is_emergency
            estop_recede = verdict.recede
            estop_hold = (
                verdict.action is EStopAction.ZERO_VELOCITY and not verdict.is_emergency
            )
            last_verdict_emergency = verdict.is_emergency
            if verdict.action is not EStopAction.NONE:
                if verdict.recede:
                    cmd = (-recede_speed, 0.0, 0.0, 0.0)
                else:
                    cmd = (0.0, 0.0, 0.0, 0.0)
                cmd_stamp = t
            elif target_fresh and latest_cam is not None:
                sp = compute_setpoint(latest_cam[0], latest_cam[1], latest_cam[2], follow_cfg)
                cmd = (sp.v_forward, sp.v_right, sp.v_down, sp.yaw_rate)
                cmd_stamp = t
            else:
                cmd = (0.0, 0.0, 0.0, 0.0)
                cmd_stamp = t

        # --- streamer at stream_hz: the real decide() ladder + hold + reflex ---
        if t >= next_stream_t:
            next_stream_t += stream_period
            reflex_range = latest_range if target_fresh else None
            holding_input = hold.is_holding and not sim_cfg.legacy_stale_semantics
            inputs = FlightInputs(
                in_guided=(mode == "GUIDED"),
                ekf_ok=sim_cfg.ekf_ok,
                command_fresh=cmd is not None and (t - cmd_stamp) <= command_stale_s,
                reflex_danger=reflex.update(
                    reflex_range, latest_rx_t if reflex_range is not None else t
                ),
                estop_emergency=estop_emergency,
                estop_recede=estop_recede,
                estop_hold=estop_hold,
                holding=holding_input,
            )
            action = decide(inputs)
            last_action_name = action.value
            result.stream_actions.append((t, action.value))

            if action is ControlAction.STREAM_VELOCITY:
                v_fwd, v_right, v_down, yaw_cmd = cmd
                speed = max(abs(v_fwd), abs(v_right), abs(v_down))
                v_fwd = slew(streamed[0], v_fwd, dv_max)
                v_right = slew(streamed[1], v_right, dv_max)
                v_down = slew(streamed[2], v_down, dv_max)
                streamed = (v_fwd, v_right, v_down)
                status = hold.step(
                    speed_mps=speed,
                    yaw_rate=yaw_cmd,
                    pose_age_s=0.0,
                    ekf_ok=sim_cfg.ekf_ok,
                    now_s=t,
                )
                if status.action is HoldAction.LATCH:
                    hold_point = (dx, dy, dz, psi)
                    result.hold_points.append((t, dx, dy, dz))
                if status.action in (HoldAction.LATCH, HoldAction.HOLD):
                    control = ("hold", (0.0, 0.0, 0.0, 0.0))
                else:
                    hold_point = None
                    control = ("velocity", (v_fwd, v_right, v_down, yaw_cmd))
            elif action is ControlAction.HOLD_POSITION:
                control = ("hold", (0.0, 0.0, 0.0, 0.0))
            else:
                hold.reset()
                hold_point = None
                streamed = (0.0, 0.0, 0.0)
                if action is ControlAction.STREAM_ZERO:
                    control = ("velocity", (0.0, 0.0, 0.0, 0.0))
                elif action is ControlAction.SET_BRAKE:
                    mode = "BRAKE"
                    fc_hold_point = (dx, dy, dz)
                    control = ("fc_hold", (0.0, 0.0, 0.0, 0.0))
                elif action is ControlAction.SET_LOITER:
                    mode = "LOITER"
                    fc_hold_point = (dx, dy, dz)
                    control = ("fc_hold", (0.0, 0.0, 0.0, 0.0))
                else:
                    control = ("fc_hold" if fc_hold_point else "velocity", (0.0, 0.0, 0.0, 0.0))

        # --- translate the active control into a world target velocity ---
        kind, body_cmd = control
        v_fwd_cmd, v_right_cmd, v_down_cmd, yaw_cmd = body_cmd
        if kind == "velocity":
            if sim_cfg.sign_error:
                v_fwd_cmd = -v_fwd_cmd
            cos_p, sin_p = math.cos(psi), math.sin(psi)
            tvx = v_fwd_cmd * cos_p - v_right_cmd * sin_p
            tvy = -v_fwd_cmd * sin_p - v_right_cmd * cos_p
            tvz = -v_down_cmd  # v_down positive = descend (-Z)
        else:
            hx, hy, hz, hpsi = (
                hold_point
                if kind == "hold" and hold_point is not None
                else (fc_hold_point + (psi,) if fc_hold_point else (dx, dy, dz, psi))
            )
            lim = sim_cfg.hold_speed_limit
            tvx = max(-lim, min(lim, sim_cfg.kp_pos * (hx - dx)))
            tvy = max(-lim, min(lim, sim_cfg.kp_pos * (hy - dy)))
            tvz = max(-lim, min(lim, sim_cfg.kp_pos * (hz - dz)))
            yaw_cmd = max(-1.0, min(1.0, 1.5 * (hpsi - psi)))
            v_fwd_cmd, v_down_cmd = 0.0, 0.0

        # --- disturbances ---
        if sim_cfg.gust_sigma > 0.0:
            for k in range(3):
                gust[k] += (-gust[k] / sim_cfg.gust_tau_s) * sim_cfg.dt + (
                    sim_cfg.gust_sigma
                    * math.sqrt(2.0 * sim_cfg.dt / sim_cfg.gust_tau_s)
                    * rng.gauss(0.0, 1.0)
                )
        residual = 1.0 - sim_cfg.wind_rejection
        dist_x = residual * (sim_cfg.wind[0] + gust[0])
        dist_y = residual * (sim_cfg.wind[1] + gust[1])
        dist_z = sim_cfg.wind[2]

        a = sim_cfg.drone_accel * sim_cfg.dt
        ax_applied = max(-a, min(a, (tvx + dist_x) - vx))
        ay_applied = max(-a, min(a, (tvy + dist_y) - vy))
        vx += ax_applied
        vy += ay_applied

        az_lim = sim_cfg.vert_accel * sim_cfg.dt
        tvz_eff = tvz + dist_z
        az_deficit = 0.0
        if sim_cfg.pitch_coupling:
            cos_p, sin_p = math.cos(psi), math.sin(psi)
            a_fwd = (ax_applied * cos_p - ay_applied * sin_p) / sim_cfg.dt
            theta_target = math.atan2(a_fwd, G)
            theta += (theta_target - theta) * (sim_cfg.dt / sim_cfg.pitch_tau_s)
            az_deficit = -G * (1.0 - math.cos(theta))
        vz += max(-az_lim, min(az_lim, tvz_eff - vz)) + az_deficit * sim_cfg.dt

        ya = sim_cfg.yaw_accel * sim_cfg.dt
        yaw_rate += max(-ya, min(ya, yaw_cmd - yaw_rate))

        dx += vx * sim_cfg.dt
        dy += vy * sim_cfg.dt
        dz += vz * sim_cfg.dt
        psi += yaw_rate * sim_cfg.dt

        result.t.append(t)
        result.drone_x.append(dx)
        result.drone_y.append(dy)
        result.drone_z.append(dz)
        result.person_x.append(px)
        result.person_y.append(py)
        result.true_range.append(true_range)
        result.v_forward_cmd.append(v_fwd_cmd)
        result.v_down_cmd.append(v_down_cmd)
        result.vz.append(vz)
        result.pitch.append(theta)
        result.v_brake_cap.append(
            0.0 if kind != "velocity" else _brake_cap_of(cmd, follow_cfg, latest_cam)
        )
        result.emergency.append(last_verdict_emergency)
        result.hold_active.append(kind != "velocity")
        result.action.append(last_action_name)

    result.relatch_count = len(result.hold_points)
    return result


def _brake_cap_of(cmd, follow_cfg, latest_cam) -> float:
    """Diagnostic braking cap for the plots (recomputed from the last detection)."""
    if latest_cam is None:
        return 0.0
    sp = compute_setpoint(latest_cam[0], latest_cam[1], latest_cam[2], follow_cfg)
    return sp.v_brake_cap

def _person(x, y, z=DRONE_Z):
    return (x, y, z)

def walk_straight(t):
    """Stand ahead, then walk slowly forward and back along the line of sight."""
    return _person(6.0 + 1.5 * math.sin(0.15 * t), 0.0)

def weave(t):
    """Walk a slow lateral S while holding roughly constant distance."""
    return _person(5.0 + 0.5 * math.sin(0.1 * t), 2.5 * math.sin(0.25 * t))

def lunge(t):
    """Approach the drone steadily, within the drone's recede capability."""
    # closes from 6 m at ~0.5 m/s, then holds just outside the ring
    return _person(max(6.0 - 0.5 * t, 1.8), 0.0)

def fast_lunge(t):
    return _person(max(4.0 - 1.6 * t, 0.6), 0.0)

def vary_height(t):
    """Walk forward/back while changing apparent height"""
    return _person(5.0 + 1.0 * math.sin(0.15 * t), 0.0, DRONE_Z + 0.8 * math.sin(0.2 * t))

def disappear(t):
    """Visible, then steps out of frame after 8 s (target-lost test)."""
    if t > 8.0:
        return None
    return _person(5.0, 0.0)

def stand_still(t):
    """Person stands fixed: the steady-state hold must engage and stick."""
    return _person(5.0, 0.0)

def hold_then_move(t):
    """Stand for 20s then walk away"""
    if t < 20.0:
        return _person(5.0, 0.0)
    return _person(5.0 + 1.0 * (t - 20.0), 0.0)

def dropout_storm(t):
    """Stationary person; heavy random detection dropout is set via SimConfig."""
    return _person(5.0, 0.0)

def gusty_follow(t):
    """walk_straight in wind; wind/gusts are set via SimConfig."""
    return walk_straight(t)

def approach_brake(t):
    """Person fixed 8 m out: one clean approach-and-brake (the descent study)."""
    return _person(8.0, 0.0)


SCENARIOS = {
    "walk_straight": walk_straight,
    "weave": weave,
    "lunge": lunge,
    "fast_lunge": fast_lunge,
    "vary_height": vary_height,
    "disappear": disappear,
    "stand_still": stand_still,
    "hold_then_move": hold_then_move,
    "dropout_storm": dropout_storm,
    "gusty_follow": gusty_follow,
    "approach_brake": approach_brake,
}

SCENARIO_SIM = {
    # wind_z is the residual sag bias; it must stay below what the vertical channel
    # corrects with < HOLD_ENTER (0.08 m/s) of standing correction, or -- by design --
    # the hold never engages (the velocity loop keeps actively correcting instead).
    "stand_still": dict(duration_s=60.0, wind=(0.03, 0.01, -0.02), gust_sigma=0.04),
    "hold_then_move": dict(duration_s=50.0, wind=(0.03, 0.01, -0.02), gust_sigma=0.04),
    "dropout_storm": dict(dropout_prob=0.25, noise_mm=40.0, seed=3),
    # drone_accel=2.0: the plant's real tracking authority exceeds the conservative
    # a_brake=1.0 the braking law assumes -- that margin is what absorbs gusts.
    # With authority == a_brake exactly, any gust toward the person mathematically
    # guarantees a breach, which is a modelling artifact, not a controller bug.
    "gusty_follow": dict(wind=(0.4, 0.2, 0.0), gust_sigma=0.3, noise_mm=30.0, seed=2, drone_accel=2.0),
    "approach_brake": dict(duration_s=30.0, pitch_coupling=True),
}

def sim_config_for(scenario: str, **overrides) -> SimConfig:
    kwargs = dict(SCENARIO_SIM.get(scenario, {}))
    kwargs.update(overrides)
    return SimConfig(**kwargs)

def _sweep(argv):
    """Descent-mystery study: sweep v_max x pitch-coupling x a_brake on approach_brake."""
    from sim.visualize import render

    rows = []
    for v_max in (0.8, 1.5):
        for coupling in (False, True):
            for a_brake in (0.6, 1.0):
                fc = FollowConfig(v_max=v_max, a_brake=a_brake)
                sc = sim_config_for("approach_brake", pitch_coupling=coupling)
                res = run_sim(SCENARIOS["approach_brake"], fc, sc)
                dip = DRONE_Z - res.min_altitude
                rows.append((v_max, coupling, a_brake, dip, res.min_true_range))
                tag = (
                    f"diag_vmax{v_max}_pitch{'on' if coupling else 'off'}_ab{a_brake}"
                ).replace(".", "p")
                render(res, title=tag, show=False)
    print(f"{'v_max':>6} {'pitch':>6} {'a_brake':>8} {'alt dip (m)':>12} {'min range':>10}")
    for v_max, coupling, a_brake, dip, min_r in rows:
        print(
            f"{v_max:>6} {('on' if coupling else 'off'):>6} {a_brake:>8} "
            f"{dip:>12.3f} {min_r:>10.2f}"
        )

def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if "--sweep" in argv:
        _sweep(argv)
        return
    scenario = "walk_straight"
    for arg in argv:
        if arg in SCENARIOS:
            scenario = arg
    result = run_sim(SCENARIOS[scenario], FollowConfig(), sim_config_for(scenario))
    print(
        f"[{scenario}] min_true_range={result.min_true_range:.2f} m "
        f"(hard_min={result.hard_min_m}), final_range={result.final_range:.2f} m "
        f"(standoff={result.standoff_m}), min_alt={result.min_altitude:.2f} m, "
        f"latches={result.relatch_count}, "
        f"vel/zero toggles={result.action_toggles()}"
    )
    if "--viz" in argv:
        from sim.visualize import render

        render(result, title=scenario)
    if "--animate" in argv:
        from sim.visualize import animate

        animate(result, title=scenario)

if __name__ == "__main__":
    main()
