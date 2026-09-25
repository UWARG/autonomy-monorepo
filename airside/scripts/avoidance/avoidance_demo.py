#!/usr/bin/env python3
"""Run native and WARG obstacle-avoidance scenarios in ArduCopter SITL.

The runner streams synthetic sectors and records JSONL results. The custom
scenario uses WARG's 2D planner to send GUIDED velocity targets.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from arm_readiness import (
    ArmReadinessSnapshot,
    StableArmReadinessGate,
    missing_arm_preconditions,
)
from harness_runtime import (
    ScenarioTimer,
    SummaryEmitter,
    WorkerFailureError,
    WorkerSupervisor,
    has_callable_attribute,
    is_flight_controller_heartbeat,
)
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

# OBSTACLE_DISTANCE (message ID 330) is a MAVLink 2 message. This must be set
# before importing pymavlink so that the generated sender exposes the method.
os.environ["MAVLINK20"] = "1"

import pymavlink
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
FAST_WORKER_STALE_S = 2.0
IO_WORKER_STALE_S = 3.0


@dataclass
class Telemetry:
    """Latest vehicle state."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0
    yaw_rad: float = 0.0
    armed: bool = False
    gps_fix: int = 0
    prearm_ok: bool = False
    ekf_using_gps: bool = False
    global_position_seen: bool = False
    local_position_seen: bool = False
    home_lat: float | None = None
    home_lon: float | None = None
    mission_requests: list[int] = field(default_factory=list)
    mission_acked: bool = False
    distance_sensor_rx: int = 0
    obstacle_tx_count: int = 0
    parameter_count: int | None = None
    parameters: dict[str, dict[str, float | int]] = field(default_factory=dict)
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
    """Return point-to-wall distance for the verdict."""
    de = max(abs(east_m) - WALL_HALF_WIDTH_M, 0.0)
    dn = WALL_NORTH_M - north_m
    return math.hypot(dn, de)


class Demo:
    def __init__(self, url: str, scenario: str) -> None:
        self.scenario = scenario
        self.wall = scenario.startswith("wall")
        self.telem = Telemetry()
        self.stop = threading.Event()
        self.supervisor = WorkerSupervisor(self.stop)
        self.scan_lock = threading.Lock()
        self.latest_scan: tuple[SectorScan, Point2D, float] | None = None
        self.planner_lock = threading.Lock()
        self.planner_status = "NOT_STARTED"
        self.planner_reason: str | None = None
        self.planner_waypoint: tuple[float, float] | None = None
        self.planner_path_found_count = 0
        self.planner_hold_count = 0
        self._monitor_metrics: dict[str, Any] = {
            "min_wall_dist_m": None,
            "breached": None,
            "max_north_m": None,
            "goal_reached_at_s": None,
        }
        self.conn = mavutil.mavlink_connection(
            url,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        print(f"[demo] waiting for heartbeat on {url} ...", flush=True)
        self._wait_for_flight_controller_heartbeat(timeout_s=120.0)
        print(
            f"[demo] heartbeat from sys {self.conn.target_system} "
            f"comp {self.conn.target_component}",
            flush=True,
        )
        if not has_callable_attribute(self.conn.mav, "obstacle_distance_send"):
            raise RuntimeError(
                "MAVLink 2 obstacle sender unavailable; "
                f"protocol={getattr(mavutil.mavlink, 'WIRE_PROTOCOL_VERSION', 'unknown')}; "
                f"dialect={getattr(mavutil, 'current_dialect', 'unknown')}; "
                f"pymavlink={getattr(pymavlink, '__version__', 'unknown')}; "
                f"target={self.conn.target_system}/{self.conn.target_component}"
            )
        # Request 10 Hz telemetry.
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

    def _wait_for_flight_controller_heartbeat(self, timeout_s: float) -> None:
        deadline_s = time.monotonic() + timeout_s
        last_rejected: tuple[int, int, int] | None = None
        while time.monotonic() < deadline_s:
            msg = self.conn.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=min(1.0, max(0.0, deadline_s - time.monotonic())),
            )
            if msg is None:
                continue
            source_system = int(msg.get_srcSystem())
            source_component = int(msg.get_srcComponent())
            autopilot = int(msg.autopilot)
            if not is_flight_controller_heartbeat(
                source_system,
                autopilot,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            ):
                last_rejected = (source_system, source_component, autopilot)
                print(
                    "[demo] ignoring non-FC heartbeat "
                    f"sys={source_system} comp={source_component} "
                    f"autopilot={autopilot}",
                    flush=True,
                )
                continue
            self.conn.target_system = source_system
            self.conn.target_component = source_component
            return
        raise TimeoutError(
            "flight-controller heartbeat timeout; "
            f"last_rejected={last_rejected}"
        )

    def start_worker(
        self,
        name: str,
        target: Callable[[], None],
        *,
        stale_after_s: float,
    ) -> None:
        self.supervisor.start(
            name,
            target,
            stale_after_s=stale_after_s,
        )

    def _check_workers(self) -> None:
        self.supervisor.check_health()

    def _sleep_checked(self, duration_s: float) -> None:
        deadline_s = time.monotonic() + duration_s
        while time.monotonic() < deadline_s:
            self._check_workers()
            time.sleep(min(0.1, max(0.0, deadline_s - time.monotonic())))

    def rx_loop(self) -> None:
        while not self.stop.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            self.supervisor.mark_progress("rx")
            kind = msg.get_type()
            t = self.telem
            if kind == "LOCAL_POSITION_NED":
                with t.lock:
                    t.north_m, t.east_m, t.down_m = msg.x, msg.y, msg.z
                    t.local_position_seen = True
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
                    if msg.lat != 0 or msg.lon != 0:
                        t.global_position_seen = True
                        if t.home_lat is None:
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
            elif kind == "PARAM_VALUE":
                parameter_id = msg.param_id
                if isinstance(parameter_id, bytes):
                    parameter_id = parameter_id.decode("ascii", errors="replace")
                name = str(parameter_id).rstrip("\x00")
                with t.lock:
                    t.parameter_count = int(msg.param_count)
                    t.parameters[name] = {
                        "value": float(msg.param_value),
                        "type": int(msg.param_type),
                        "index": int(msg.param_index),
                    }
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
            self.supervisor.mark_progress("heartbeat")
            time.sleep(1.0)

    def obstacle_loop(self) -> None:
        """Stream synthetic obstacle sectors."""
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
            with self.telem.lock:
                self.telem.obstacle_tx_count += 1
            self.supervisor.mark_progress("obstacle")
            time.sleep(period)

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
        self._sleep_checked(1.0)  # PARAM_VALUE echo is consumed by the rx thread
        print(f"[demo] set {name}={value}", flush=True)

    def set_mode(self, name: str) -> None:
        mode_id = self.conn.mode_mapping()[name]
        self.command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def _arm_readiness_state(
        self,
    ) -> tuple[ArmReadinessSnapshot, tuple[str, ...]]:
        with self.telem.lock:
            snapshot = ArmReadinessSnapshot(
                gps_fix=self.telem.gps_fix,
                ekf_using_gps=self.telem.ekf_using_gps,
                home_position_seen=self.telem.global_position_seen,
                local_position_seen=self.telem.local_position_seen,
                prearm_ok=self.telem.prearm_ok,
            )
            recent_status = tuple(self.telem.statustexts[-5:])
        return snapshot, recent_status

    def wait_ready_to_arm(
        self,
        timeout_s: float = 180.0,
        stable_s: float = 5.0,
    ) -> None:
        """Wait until every arm prerequisite remains stable."""

        deadline = time.monotonic() + timeout_s
        gate = StableArmReadinessGate(stable_s=stable_s)
        reported_missing: tuple[str, ...] | None = None
        while time.monotonic() < deadline:
            self._check_workers()
            now_s = time.monotonic()
            snapshot, _ = self._arm_readiness_state()
            ready = gate.update(now_s, snapshot)
            if gate.last_missing != reported_missing:
                reported_missing = gate.last_missing
                if reported_missing:
                    print(
                        "[demo] waiting for arm readiness: "
                        + ", ".join(reported_missing),
                        flush=True,
                    )
                else:
                    print(
                        f"[demo] arm prerequisites satisfied; "
                        f"stabilizing for {stable_s:.1f}s",
                        flush=True,
                    )
            if ready:
                print("[demo] arm readiness stable", flush=True)
                return
            self._sleep_checked(0.5)

        snapshot, recent_status = self._arm_readiness_state()
        missing = missing_arm_preconditions(snapshot)
        raise TimeoutError(
            "arm readiness timeout; "
            f"missing={missing}; snapshot={snapshot}; "
            f"recent_status={recent_status}"
        )

    def arm(self, timeout_s: float = 60.0) -> None:
        snapshot, _ = self._arm_readiness_state()
        if missing_arm_preconditions(snapshot):
            self.wait_ready_to_arm(timeout_s=timeout_s)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_workers()
            snapshot, recent_status = self._arm_readiness_state()
            missing = missing_arm_preconditions(snapshot)
            if missing:
                remaining_s = max(0.0, deadline - time.monotonic())
                if remaining_s <= 0.0:
                    break
                self.wait_ready_to_arm(timeout_s=remaining_s)
                snapshot, recent_status = self._arm_readiness_state()
                missing = missing_arm_preconditions(snapshot)
                if missing:
                    raise RuntimeError(
                        "arm prerequisites disappeared before command; "
                        f"missing={missing}; snapshot={snapshot}; "
                        f"recent_status={recent_status}"
                    )
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            for _ in range(30):
                self._check_workers()
                with self.telem.lock:
                    if self.telem.armed:
                        print("[demo] armed", flush=True)
                        return
                self._sleep_checked(0.1)
        snapshot, recent_status = self._arm_readiness_state()
        missing = missing_arm_preconditions(snapshot)
        raise TimeoutError(
            "failed to arm; "
            f"missing={missing}; snapshot={snapshot}; "
            f"recent_status={recent_status}"
        )

    def takeoff(self, alt_m: float, timeout_s: float = 90.0) -> None:
        """Arm and retry takeoff until altitude."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_workers()
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
            self._sleep_checked(3.0)
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
        """Stream local-NED velocity at 10 Hz."""
        while not self.stop.is_set():
            self.send_velocity(vx_mps, 0.0)
            self.supervisor.mark_progress("velocity")
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
        """Plan from sectors and stream safe velocity."""
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
                self.supervisor.mark_progress("planner")
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
                self.supervisor.mark_progress("planner")
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
            self.supervisor.mark_progress("planner")
            time.sleep(0.1)

    def _record_planner_hold(self, reason: str) -> None:
        with self.planner_lock:
            self.planner_status = PlanStatus.NO_PATH.value
            self.planner_reason = reason
            self.planner_waypoint = None
            self.planner_hold_count += 1

    def upload_goal_mission(self) -> None:
        """Upload takeoff and goal waypoints."""
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
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            self._check_workers()
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
            self._sleep_checked(0.05)
        raise TimeoutError("mission upload not acknowledged")

    def download_parameters(self, output_path: str, timeout_s: float = 60.0) -> None:
        """Download and persist one complete effective FC parameter set."""

        with self.telem.lock:
            self.telem.parameter_count = None
            self.telem.parameters.clear()
        self.conn.mav.param_request_list_send(
            self.conn.target_system,
            self.conn.target_component,
        )
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            self._check_workers()
            with self.telem.lock:
                expected = self.telem.parameter_count
                received = len(self.telem.parameters)
                complete = expected is not None and expected > 0 and received >= expected
                parameters = dict(self.telem.parameters) if complete else None
            if parameters is not None:
                payload = {
                    "expected_count": expected,
                    "received_count": received,
                    "parameters": dict(sorted(parameters.items())),
                }
                destination = Path(output_path)
                destination.parent.mkdir(parents=True, exist_ok=True)
                temporary = destination.with_suffix(destination.suffix + ".tmp")
                temporary.write_text(
                    json.dumps(payload, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                temporary.replace(destination)
                print(
                    f"[demo] downloaded {received}/{expected} parameters",
                    flush=True,
                )
                return
            self._sleep_checked(0.1)
        with self.telem.lock:
            expected = self.telem.parameter_count
            received = len(self.telem.parameters)
        raise TimeoutError(
            "parameter download incomplete; "
            f"received={received}; expected={expected}"
        )

    def monitor(self, duration_s: float, log_path: str) -> dict:
        timer = ScenarioTimer()
        min_wall_dist = math.inf
        max_north = -math.inf
        goal_reached_at = None
        breached = False
        prev_n: float | None = None
        prev_e = 0.0
        with open(log_path, "w", encoding="utf-8") as log:
            while not timer.expired(duration_s):
                self._check_workers()
                elapsed_s = timer.elapsed_s()
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
                wall_d = distance_to_wall_m(n, e) if self.wall else None
                if wall_d is not None:
                    min_wall_dist = min(min_wall_dist, wall_d)
                    # Detect crossings through the wall segment.
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
                    goal_reached_at = round(elapsed_s, 1)
                self._monitor_metrics = {
                    "min_wall_dist_m": (
                        round(min_wall_dist, 2)
                        if self.wall and min_wall_dist < math.inf
                        else None
                    ),
                    "breached": breached if self.wall else None,
                    "max_north_m": round(max_north, 2),
                    "goal_reached_at_s": goal_reached_at,
                }
                log.write(
                    json.dumps(
                        {
                            "t": round(elapsed_s, 2),
                            "utc": datetime.now(timezone.utc).isoformat(),
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
                self._sleep_checked(0.5)
        return self.summary_snapshot()

    def summary_snapshot(self) -> dict[str, Any]:
        """Return the latest telemetry, planner, and monitor metrics."""

        with self.telem.lock:
            armed = self.telem.armed
            distance_sensor_rx = self.telem.distance_sensor_rx
            obstacle_tx_count = self.telem.obstacle_tx_count
            north_m = self.telem.north_m
        with self.planner_lock:
            planner_status = self.planner_status
            planner_reason = self.planner_reason
            planner_path_found_count = self.planner_path_found_count
            planner_hold_count = self.planner_hold_count
        summary = dict(self._monitor_metrics)
        if summary["max_north_m"] is None:
            summary["max_north_m"] = round(north_m, 2)
        summary.update(
            {
                "armed": armed,
                "distance_sensor_rx": distance_sensor_rx,
                "obstacle_tx_count": obstacle_tx_count,
                "planner_status": planner_status,
                "planner_reason": planner_reason,
                "planner_path_found_count": planner_path_found_count,
                "planner_hold_count": planner_hold_count,
            }
        )
        return summary

    def shutdown(self) -> None:
        self.supervisor.stop_and_join()
        self.conn.close()


def empty_summary(scenario: str) -> dict[str, Any]:
    return {
        "scenario": scenario,
        "verdict": "FAIL",
        "armed": False,
        "failure_stage": None,
        "failure_reason": None,
        "worker_failure": None,
        "min_wall_dist_m": None,
        "breached": None,
        "max_north_m": None,
        "goal_reached_at_s": None,
        "distance_sensor_rx": 0,
        "obstacle_tx_count": 0,
        "planner_status": "NOT_STARTED",
        "planner_reason": None,
        "planner_path_found_count": 0,
        "planner_hold_count": 0,
        "clearance_ok": None,
    }


def apply_verdict(summary: dict[str, Any], scenario: str) -> None:
    """Apply scenario-specific acceptance rules in place."""

    summary["scenario"] = scenario
    summary["failure_stage"] = None
    summary["failure_reason"] = None
    summary["worker_failure"] = None
    if scenario.startswith("wall"):
        minimum = summary["min_wall_dist_m"]
        summary["clearance_ok"] = (
            minimum is not None and minimum >= MIN_CLEARANCE_M
        )
    else:
        summary["clearance_ok"] = None

    if scenario == "wall_custom_2d":
        qualified = (
            summary["armed"]
            and not summary["breached"]
            and summary["clearance_ok"]
            and summary["goal_reached_at_s"] is not None
            and summary["distance_sensor_rx"] > 0
            and summary["obstacle_tx_count"] > 0
            and summary["planner_path_found_count"] > 0
            and summary["planner_hold_count"] == 0
            and summary["planner_status"] == PlanStatus.PATH_FOUND.value
        )
    elif scenario in ("wall_guided_wpnav", "wall_auto"):
        qualified = (
            not summary["breached"]
            and summary["clearance_ok"]
            and summary["goal_reached_at_s"] is not None
        )
    elif scenario in ("wall_guided", "wall_guided_vel"):
        qualified = not summary["breached"]
    else:
        qualified = summary["goal_reached_at_s"] is not None
    summary["verdict"] = "PASS" if qualified else "FAIL"


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
    parser.add_argument("--summary-json", default=None)
    parser.add_argument("--params-json", default=None)
    args = parser.parse_args()
    log_path = args.log or f"/demo/logs/{args.scenario}.jsonl"
    emitter = SummaryEmitter(args.summary_json)
    summary = empty_summary(args.scenario)
    demo: Demo | None = None
    stage = "connect"
    try:
        demo = Demo(args.url, args.scenario)
        stage = "worker_startup"
        demo.start_worker("rx", demo.rx_loop, stale_after_s=IO_WORKER_STALE_S)
        demo.start_worker(
            "heartbeat",
            demo.heartbeat_loop,
            stale_after_s=IO_WORKER_STALE_S,
        )
        demo.start_worker(
            "obstacle",
            demo.obstacle_loop,
            stale_after_s=FAST_WORKER_STALE_S,
        )
        if args.params_json:
            stage = "parameter_download"
            demo.download_parameters(args.params_json)
        stage = "readiness"
        demo.wait_ready_to_arm()
        stage = "configuration"
        if args.scenario == "wall_guided_wpnav":
            demo.set_param("GUID_OPTIONS", 64)
        if args.scenario == "wall_custom_2d":
            # The WARG planner owns avoidance.
            demo.set_param("AVOID_ENABLE", 0)
        if args.scenario == "wall_auto":
            stage = "mission_upload"
            demo.upload_goal_mission()
            stage = "arm"
            demo.set_mode("AUTO")
            demo.arm()
            # The mission starts on arming.
        elif args.scenario == "wall_guided_vel":
            stage = "takeoff"
            demo.set_mode("GUIDED")
            demo.takeoff(ALT_M)
            stage = "velocity_startup"
            demo.start_worker(
                "velocity",
                lambda: demo.velocity_stream_loop(2.0),
                stale_after_s=FAST_WORKER_STALE_S,
            )
            print("[demo] streaming 2 m/s north velocity setpoints", flush=True)
        elif args.scenario == "wall_custom_2d":
            stage = "takeoff"
            demo.set_mode("GUIDED")
            demo.takeoff(ALT_M)
            stage = "planner_startup"
            demo.start_worker(
                "planner",
                demo.custom_planner_loop,
                stale_after_s=FAST_WORKER_STALE_S,
            )
            print("[demo] WARG 2D planner owns velocity setpoints", flush=True)
        else:
            stage = "takeoff"
            demo.set_mode("GUIDED")
            demo.takeoff(ALT_M)
            stage = "goto"
            demo.goto_local(GOAL_NORTH_M, 0.0, ALT_M)
            print(f"[demo] goto {GOAL_NORTH_M} m north sent", flush=True)

        stage = "monitor"
        summary = demo.monitor(args.duration, log_path)
        apply_verdict(summary, args.scenario)
    except Exception as exc:  # noqa: BLE001 - scenario boundary
        traceback.print_exc()
        if demo is not None:
            summary.update(demo.summary_snapshot())
            failure = demo.supervisor.failure()
            if isinstance(exc, WorkerFailureError):
                failure = exc.failure
            summary["worker_failure"] = (
                failure.to_dict() if failure is not None else None
            )
        summary.update(
            {
                "scenario": args.scenario,
                "verdict": "FAIL",
                "failure_stage": stage,
                "failure_reason": f"{type(exc).__name__}: {exc}",
            }
        )
    finally:
        if demo is not None:
            try:
                demo.shutdown()
            except Exception as exc:  # noqa: BLE001 - preserve summary
                traceback.print_exc()
                if summary["verdict"] != "FAIL":
                    summary.update(
                        {
                            "verdict": "FAIL",
                            "failure_stage": "shutdown",
                            "failure_reason": f"{type(exc).__name__}: {exc}",
                        }
                    )
    emitter.emit(summary)
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
