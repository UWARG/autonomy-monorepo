#!/usr/bin/env python3
"""
Issue 96 -- obstacle avoidance integration demo.

Proves the companion-computer integration pattern end-to-end in SITL:

    obstacle source (here: a synthetic wall; later: OAK-D / LiDAR driver)
        -> MAVLink OBSTACLE_DISTANCE @ 10 Hz          (this script)
        -> ArduPilot proximity library (PRX1_TYPE=2)
        -> AC_Avoid + BendyRuler (AVOID_ENABLE=7, OA_TYPE=1)

The synthetic obstacle is a finite vertical wall segment fixed in the world.
Each tick the script ray-casts the 72 five-degree body-frame sectors against
the wall from the vehicle's current position/yaw -- exactly the shape of
message a real depth-camera bridge would emit -- so swapping in real hardware
later only replaces `wall_sector_distances()`.

Scenarios (run inside the SITL container, one fresh boot per run):
    clear_guided       no obstacle; GUIDED goto 40 m north. Baseline.
    wall_guided        wall at 20 m north; same GUIDED goto. Documents the
                       gap: plain SET_POSITION_TARGET bypasses BendyRuler.
    wall_guided_wpnav  same, with GUID_OPTIONS=64 (position targets routed
                       through WPNav, which carries the OA wrapper).
    wall_guided_vel    wall at 20 m north; GUIDED *velocity* setpoints
                       streamed at 10 Hz toward the wall -- the control path
                       the follow stack uses. Tests AC_Avoid velocity limiting.
    wall_auto          wall at 20 m north; 1-waypoint AUTO mission through it.
    wall_custom_2d     wall at 20 m north; WARG's pure 2D planner consumes the
                       same sectors and streams safe GUIDED velocity targets.

Each run writes a JSONL telemetry log and prints a PASS/FAIL verdict:
the vehicle must never come closer than MIN_CLEARANCE_M to the wall, and
(secondary, "stay on task") ideally still reaches the goal.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass, field

from obstacle_avoidance import (
    BendyRuler2D,
    ObstacleSnapshot,
    PlannerConfig,
    PlanRequest,
    PlanStatus,
    Point2D,
    SectorScan,
    sector_scan_to_snapshot,
)
from pymavlink import mavutil

SEND_HZ = 10.0
SECTORS = 72
INCREMENT_DEG = 5
MIN_RANGE_CM = 30
MAX_RANGE_CM = 2000
CLEAR_CM = MAX_RANGE_CM + 1  # per spec: > max_distance means "no obstacle"

WALL_NORTH_M = 20.0
WALL_HALF_WIDTH_M = 6.0
GOAL_NORTH_M = 40.0
ALT_M = 10.0
MIN_CLEARANCE_M = 1.0  # hard verdict floor (OA_MARGIN_MAX is 3 m)
GOAL_TOLERANCE_M = 2.0
CUSTOM_PLANNER_SPEED_MPS = 2.0
CUSTOM_OBSTACLE_RADIUS_M = 0.75


@dataclass
class Telemetry:
    """Latest vehicle state, updated by the RX thread."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0
    yaw_rad: float = 0.0
    armed: bool = False
    gps_fix: int = 0
    prearm_ok: bool = False
    ekf_using_gps: bool = False
    home_lat: float | None = None
    home_lon: float | None = None
    mission_requests: list[int] = field(default_factory=list)
    mission_acked: bool = False
    distance_sensor_rx: int = 0
    statustexts: list[str] = field(default_factory=list)


def wall_sector_distances(
    north_m: float, east_m: float, yaw_rad: float, wall: bool
) -> list[int]:
    """Ray-cast the 72 body-frame sectors against the wall segment (cm)."""
    distances = [CLEAR_CM] * SECTORS
    if not wall:
        return distances
    for i in range(SECTORS):
        angle = yaw_rad + math.radians(i * INCREMENT_DEG)
        dir_n, dir_e = math.cos(angle), math.sin(angle)
        if dir_n <= 1e-6:  # ray parallel to or away from the wall plane
            continue
        t = (WALL_NORTH_M - north_m) / dir_n
        if t <= 0:
            continue
        hit_e = east_m + t * dir_e
        if abs(hit_e) > WALL_HALF_WIDTH_M:
            continue  # misses the finite segment
        dist_cm = int(t * 100.0)
        if dist_cm <= MAX_RANGE_CM:
            distances[i] = max(dist_cm, MIN_RANGE_CM)
    return distances


def distance_to_wall_m(north_m: float, east_m: float) -> float:
    """True point-to-segment distance, for the verdict (not sent to the FC)."""
    de = max(abs(east_m) - WALL_HALF_WIDTH_M, 0.0)
    dn = WALL_NORTH_M - north_m
    return math.hypot(dn, de)


class Demo:
    def __init__(self, url: str, scenario: str) -> None:
        self.scenario = scenario
        self.wall = scenario.startswith("wall")
        self.telem = Telemetry()
        self.stop = threading.Event()
        self.scan_lock = threading.Lock()
        self.latest_scan: tuple[SectorScan, Point2D, float] | None = None
        self.planner_lock = threading.Lock()
        self.planner_status = "NOT_STARTED"
        self.planner_reason: str | None = None
        self.planner_waypoint: tuple[float, float] | None = None
        self.planner_path_found_count = 0
        self.planner_hold_count = 0
        self.conn = mavutil.mavlink_connection(
            url,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        print(f"[demo] waiting for heartbeat on {url} ...", flush=True)
        self.conn.wait_heartbeat(timeout=120)
        print(
            f"[demo] heartbeat from sys {self.conn.target_system} "
            f"comp {self.conn.target_component}",
            flush=True,
        )
        # Bare SITL streams nothing until asked: request all groups at 10 Hz.
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

    # ------------------------------------------------------------- threads
    def rx_loop(self) -> None:
        while not self.stop.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            kind = msg.get_type()
            t = self.telem
            if kind == "LOCAL_POSITION_NED":
                with t.lock:
                    t.north_m, t.east_m, t.down_m = msg.x, msg.y, msg.z
            elif kind == "ATTITUDE":
                with t.lock:
                    t.yaw_rad = msg.yaw
            elif kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                with t.lock:
                    t.armed = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
            elif kind == "GLOBAL_POSITION_INT":
                with t.lock:
                    if t.home_lat is None and msg.lat != 0:
                        t.home_lat = msg.lat / 1e7
                        t.home_lon = msg.lon / 1e7
            elif kind == "GPS_RAW_INT":
                with t.lock:
                    t.gps_fix = msg.fix_type
            elif kind == "SYS_STATUS":
                bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
                with t.lock:
                    t.prearm_ok = bool(msg.onboard_control_sensors_health & bit)
            elif kind in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                with t.lock:
                    t.mission_requests.append(msg.seq)
            elif kind == "MISSION_ACK":
                with t.lock:
                    t.mission_acked = True
            elif kind == "DISTANCE_SENSOR":
                with t.lock:
                    t.distance_sensor_rx += 1
            elif kind == "STATUSTEXT":
                with t.lock:
                    t.statustexts.append(msg.text)
                    if "is using GPS" in msg.text:
                        t.ekf_using_gps = True
                print(f"[fc] {msg.text}", flush=True)

    def heartbeat_loop(self) -> None:
        while not self.stop.is_set():
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            time.sleep(1.0)

    def obstacle_loop(self) -> None:
        """The bridge: what a real depth-camera driver would publish."""
        period = 1.0 / SEND_HZ
        while not self.stop.is_set():
            with self.telem.lock:
                n, e, yaw = (
                    self.telem.north_m,
                    self.telem.east_m,
                    self.telem.yaw_rad,
                )
            distances = wall_sector_distances(n, e, yaw, self.wall)
            captured_at = time.monotonic()
            with self.scan_lock:
                self.latest_scan = (
                    SectorScan(
                        ranges_m=tuple(
                            None
                            if distance_cm > MAX_RANGE_CM
                            else distance_cm / 100.0
                            for distance_cm in distances
                        ),
                        angle_offset_rad=0.0,
                        angle_increment_rad=math.radians(INCREMENT_DEG),
                        timestamp_s=captured_at,
                    ),
                    Point2D(n, e),
                    yaw,
                )
            self.conn.mav.obstacle_distance_send(
                int(time.time() * 1e6),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                distances,
                INCREMENT_DEG,
                MIN_RANGE_CM,
                MAX_RANGE_CM,
                0.0,  # increment_f: 0 -> use integer increment
                0.0,  # angle_offset: sector 0 = straight ahead
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
            )
            time.sleep(period)

    # ------------------------------------------------------------ commands
    def command(self, cmd: int, *params: float) -> None:
        args = list(params) + [0.0] * (7 - len(params))
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            cmd,
            0,
            *args,
        )

    def set_param(self, name: str, value: float) -> None:
        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            name.encode("ascii"),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        time.sleep(1.0)  # PARAM_VALUE echo is consumed by the rx thread
        print(f"[demo] set {name}={value}", flush=True)

    def set_mode(self, name: str) -> None:
        mode_id = self.conn.mode_mapping()[name]
        self.command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def wait_ready_to_arm(self, timeout_s: float = 180.0) -> None:
        """Block until 3D GPS fix + EKF on GPS + home position known.

        (SYS_STATUS's prearm bit is not consistently reported by SITL, so
        readiness is judged from GPS/EKF state; the takeoff retry loop
        backstops any remaining prearm rejection.)
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.telem.lock:
                ready = (
                    self.telem.gps_fix >= 3
                    and self.telem.ekf_using_gps
                    and self.telem.home_lat is not None
                ) or self.telem.prearm_ok
            if ready:
                print("[demo] EKF/GPS ready", flush=True)
                time.sleep(5.0)  # settle margin before arming
                return
            time.sleep(0.5)
        raise TimeoutError("EKF/GPS never became ready")

    def arm(self, timeout_s: float = 60.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            for _ in range(30):
                with self.telem.lock:
                    if self.telem.armed:
                        print("[demo] armed", flush=True)
                        return
                time.sleep(0.1)
        raise TimeoutError("failed to arm")

    def takeoff(self, alt_m: float, timeout_s: float = 90.0) -> None:
        """Arm + NAV_TAKEOFF, re-issuing both if the FC auto-disarms."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.telem.lock:
                armed = self.telem.armed
                alt = -self.telem.down_m
            if alt >= alt_m * 0.9:
                print(f"[demo] at altitude {alt:.1f} m", flush=True)
                return
            if not armed:
                self.arm()
            self.command(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m
            )
            time.sleep(3.0)
        raise TimeoutError("takeoff did not reach altitude")

    def goto_local(self, north_m: float, east_m: float, alt_m: float) -> None:
        type_mask = 0x0DF8  # use position only
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north_m,
            east_m,
            -alt_m,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    def velocity_stream_loop(self, vx_mps: float) -> None:
        """Stream body-agnostic NED velocity setpoints at 10 Hz (follow-stack style)."""
        while not self.stop.is_set():
            self.send_velocity(vx_mps, 0.0)
            time.sleep(0.1)

    def send_velocity(self, north_mps: float, east_mps: float) -> None:
        """Send one local-NED horizontal velocity target."""
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0x0DC7,  # use velocity only
            0,
            0,
            0,
            north_mps,
            east_mps,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def custom_planner_loop(self) -> None:
        """Adapt sector data to the pure planner and stream its safe command."""
        planner = BendyRuler2D(
            PlannerConfig(
                first_lookahead_m=8.0,
                second_lookahead_m=8.0,
                clearance_margin_m=1.0,
                map_freshness_s=0.3,
                hysteresis_cost_m=0.75,
            )
        )
        goal = Point2D(GOAL_NORTH_M, 0.0)
        while not self.stop.is_set():
            now = time.monotonic()
            with self.telem.lock:
                position = Point2D(self.telem.north_m, self.telem.east_m)
            with self.scan_lock:
                captured_scan = self.latest_scan

            if captured_scan is None:
                self._record_planner_hold("NO_SCAN")
                self.send_velocity(0.0, 0.0)
                time.sleep(0.1)
                continue

            scan, scan_position, scan_yaw = captured_scan
            snapshot = sector_scan_to_snapshot(
                scan,
                sensor_position=scan_position,
                sensor_heading_rad=scan_yaw,
                obstacle_radius_m=CUSTOM_OBSTACLE_RADIUS_M,
            )
            result = planner.plan(
                PlanRequest(
                    start=position,
                    goal=goal,
                    obstacles=ObstacleSnapshot(
                        obstacles=snapshot.obstacles,
                        timestamp_s=snapshot.timestamp_s,
                        healthy=snapshot.healthy,
                    ),
                    now_s=now,
                )
            )
            if result.status is PlanStatus.NO_PATH or result.waypoint is None:
                reason = result.reason.value if result.reason is not None else "NO_PATH"
                self._record_planner_hold(reason)
                self.send_velocity(0.0, 0.0)
                time.sleep(0.1)
                continue

            delta_north = result.waypoint.x - position.x
            delta_east = result.waypoint.y - position.y
            distance_m = math.hypot(delta_north, delta_east)
            if distance_m <= GOAL_TOLERANCE_M:
                self.send_velocity(0.0, 0.0)
            else:
                speed = min(CUSTOM_PLANNER_SPEED_MPS, distance_m)
                self.send_velocity(
                    speed * delta_north / distance_m,
                    speed * delta_east / distance_m,
                )
            with self.planner_lock:
                self.planner_status = result.status.value
                self.planner_reason = None
                self.planner_waypoint = (result.waypoint.x, result.waypoint.y)
                self.planner_path_found_count += 1
            time.sleep(0.1)

    def _record_planner_hold(self, reason: str) -> None:
        with self.planner_lock:
            self.planner_status = PlanStatus.NO_PATH.value
            self.planner_reason = reason
            self.planner_waypoint = None
            self.planner_hold_count += 1

    def upload_goal_mission(self) -> None:
        """Two-item mission: takeoff, then a waypoint GOAL_NORTH_M north."""
        with self.telem.lock:
            lat, lon = self.telem.home_lat, self.telem.home_lon
        if lat is None:
            raise RuntimeError("no GLOBAL_POSITION_INT yet; cannot build mission")
        goal_lat = lat + GOAL_NORTH_M / 111_111.0

        def item(seq: int, cmd: int, p7: float, ilat: float, ilon: float):
            self.conn.mav.mission_item_int_send(
                self.conn.target_system,
                self.conn.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                cmd,
                0,
                1,
                0, 0, 0, 0,
                int(ilat * 1e7),
                int(ilon * 1e7),
                p7,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )

        self.conn.mav.mission_count_send(
            self.conn.target_system,
            self.conn.target_component,
            3,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        sent: set[int] = set()
        deadline = time.time() + 30
        while time.time() < deadline:
            with self.telem.lock:
                reqs = [s for s in self.telem.mission_requests if s not in sent]
                acked = self.telem.mission_acked
            if acked:
                print("[demo] mission uploaded", flush=True)
                return
            for seq in reqs:
                if seq == 0:
                    item(0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, lat, lon)
                elif seq == 1:
                    item(1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, ALT_M, 0, 0)
                else:
                    item(2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, ALT_M, goal_lat, lon)
                sent.add(seq)
            time.sleep(0.05)
        raise TimeoutError("mission upload not acknowledged")

    # ------------------------------------------------------------- monitor
    def monitor(self, duration_s: float, log_path: str) -> dict:
        start = time.time()
        min_wall_dist = math.inf
        max_north = -math.inf
        goal_reached_at = None
        breached = False
        prev_n: float | None = None
        prev_e = 0.0
        with open(log_path, "w", encoding="utf-8") as log:
            while time.time() - start < duration_s:
                with self.telem.lock:
                    n, e, d = (
                        self.telem.north_m,
                        self.telem.east_m,
                        self.telem.down_m,
                    )
                    ds_rx = self.telem.distance_sensor_rx
                with self.planner_lock:
                    planner_status = self.planner_status
                    planner_reason = self.planner_reason
                    planner_waypoint = self.planner_waypoint
                    planner_path_found_count = self.planner_path_found_count
                    planner_hold_count = self.planner_hold_count
                wall_d = distance_to_wall_m(n, e) if self.wall else None
                if wall_d is not None:
                    min_wall_dist = min(min_wall_dist, wall_d)
                    # Impact = the path crossed the wall plane *through* the
                    # segment (interpolate east at the crossing; flying around
                    # the edge and passing behind the wall is legitimate).
                    if (
                        prev_n is not None
                        and prev_n < WALL_NORTH_M <= n
                        and n > prev_n
                    ):
                        frac = (WALL_NORTH_M - prev_n) / (n - prev_n)
                        e_cross = prev_e + frac * (e - prev_e)
                        if abs(e_cross) <= WALL_HALF_WIDTH_M:
                            breached = True
                    prev_n, prev_e = n, e
                max_north = max(max_north, n)
                if (
                    goal_reached_at is None
                    and abs(n - GOAL_NORTH_M) < GOAL_TOLERANCE_M
                    and abs(e) < 10.0
                ):
                    goal_reached_at = round(time.time() - start, 1)
                log.write(
                    json.dumps(
                        {
                            "t": round(time.time() - start, 2),
                            "north_m": round(n, 2),
                            "east_m": round(e, 2),
                            "alt_m": round(-d, 2),
                            "wall_dist_m": (
                                round(wall_d, 2) if wall_d is not None else None
                            ),
                            "distance_sensor_rx": ds_rx,
                            "planner_status": planner_status,
                            "planner_reason": planner_reason,
                            "planner_waypoint": planner_waypoint,
                        }
                    )
                    + "\n"
                )
                if goal_reached_at is not None:
                    break
                time.sleep(0.5)
        return {
            "min_wall_dist_m": (
                round(min_wall_dist, 2) if self.wall else None
            ),
            "breached": breached if self.wall else None,
            "max_north_m": round(max_north, 2),
            "goal_reached_at_s": goal_reached_at,
            "distance_sensor_rx": ds_rx,
            "planner_status": planner_status,
            "planner_reason": planner_reason,
            "planner_path_found_count": planner_path_found_count,
            "planner_hold_count": planner_hold_count,
        }

    def shutdown(self) -> None:
        self.stop.set()
        time.sleep(0.3)
        self.conn.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=[
            "clear_guided",
            "wall_guided",
            "wall_guided_wpnav",
            "wall_guided_vel",
            "wall_auto",
            "wall_custom_2d",
        ],
        required=True,
    )
    parser.add_argument("--url", default="tcp:127.0.0.1:5760")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--log", default=None)
    args = parser.parse_args()
    log_path = args.log or f"/demo/logs/{args.scenario}.jsonl"

    demo = Demo(args.url, args.scenario)
    for target in (demo.rx_loop, demo.heartbeat_loop, demo.obstacle_loop):
        threading.Thread(target=target, daemon=True).start()

    try:
        demo.wait_ready_to_arm()
        if args.scenario == "wall_guided_wpnav":
            demo.set_param("GUID_OPTIONS", 64)
        if args.scenario == "wall_custom_2d":
            # The companion planner owns avoidance for this scenario.
            demo.set_param("AVOID_ENABLE", 0)
        if args.scenario == "wall_auto":
            demo.upload_goal_mission()
            demo.set_mode("AUTO")
            demo.arm()
            # AUTO_OPTIONS=3: mission (takeoff + waypoint) starts on arming.
        elif args.scenario == "wall_guided_vel":
            demo.set_mode("GUIDED")
            demo.takeoff(ALT_M)
            threading.Thread(
                target=demo.velocity_stream_loop, args=(2.0,), daemon=True
            ).start()
            print("[demo] streaming 2 m/s north velocity setpoints", flush=True)
        elif args.scenario == "wall_custom_2d":
            demo.set_mode("GUIDED")
            demo.takeoff(ALT_M)
            threading.Thread(target=demo.custom_planner_loop, daemon=True).start()
            print("[demo] WARG 2D planner owns velocity setpoints", flush=True)
        else:
            demo.set_mode("GUIDED")
            demo.takeoff(ALT_M)
            demo.goto_local(GOAL_NORTH_M, 0.0, ALT_M)
            print(f"[demo] goto {GOAL_NORTH_M} m north sent", flush=True)

        summary = demo.monitor(args.duration, log_path)
    finally:
        demo.shutdown()

    summary["scenario"] = args.scenario
    if demo.wall:
        # Primary: never hit the wall. Secondary (reported, not gating):
        # kept the intended clearance and/or still reached the goal.
        summary["clearance_ok"] = summary["min_wall_dist_m"] >= MIN_CLEARANCE_M
        if args.scenario == "wall_custom_2d":
            qualified = (
                not summary["breached"]
                and summary["clearance_ok"]
                and summary["goal_reached_at_s"] is not None
                and summary["planner_path_found_count"] > 0
            )
            summary["verdict"] = "PASS" if qualified else "FAIL"
        else:
            summary["verdict"] = "FAIL" if summary["breached"] else "PASS"
    else:
        reached = summary["goal_reached_at_s"] is not None
        summary["verdict"] = "PASS" if reached else "FAIL"
    print(f"[demo] summary: {json.dumps(summary)}", flush=True)
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
