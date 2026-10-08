"""Atomic assembly and freshness checks for obstacle snapshot messages."""

from __future__ import annotations

import dataclasses


@dataclasses.dataclass(frozen=True)
class ObstacleRecord:
    sequence: int
    aircraft_index: int
    latitude_deg: float
    longitude_deg: float
    altitude_agl_m: float
    horizontal_keep_away_m: float
    vertical_keep_away_m: float
    speed_mps: float
    direction_deg: float


def altitude_ranges_overlap(
    *,
    vehicle_altitude_agl_m: float,
    target_altitude_agl_m: float,
    obstacle_altitude_agl_m: float,
    vertical_keep_away_m: float,
) -> bool:
    """Whether a climb/descent interval enters an obstacle's vertical band."""

    path_low = min(vehicle_altitude_agl_m, target_altitude_agl_m)
    path_high = max(vehicle_altitude_agl_m, target_altitude_agl_m)
    obstacle_low = obstacle_altitude_agl_m - vertical_keep_away_m
    obstacle_high = obstacle_altitude_agl_m + vertical_keep_away_m
    return path_low <= obstacle_high and obstacle_low <= path_high


class TrafficStore:
    """Commit a snapshot only when its status and every listed obstacle arrive."""

    def __init__(self, *, own_aircraft_index: int, freshness_s: float) -> None:
        if not 0 <= own_aircraft_index <= 255:
            raise ValueError("own aircraft index must fit uint8")
        if freshness_s <= 0.0:
            raise ValueError("traffic freshness must be positive")
        self.own_aircraft_index = own_aircraft_index
        self.freshness_s = freshness_s
        self._pending: dict[int, dict[int, ObstacleRecord]] = {}
        self._expected: dict[int, tuple[int, ...]] = {}
        self._latest_sequence = -1
        self._latest_received_s: float | None = None
        self._latest_healthy = False
        self._latest_reason = "WAITING_FOR_TRAFFIC"
        self._active: tuple[ObstacleRecord, ...] = ()
        self._active_sequence = -1

    def receive_obstacle(self, obstacle: ObstacleRecord) -> None:
        if obstacle.sequence < self._latest_sequence:
            return
        self._pending.setdefault(obstacle.sequence, {})[obstacle.aircraft_index] = obstacle
        self._try_commit(obstacle.sequence)

    def receive_status(
        self,
        *,
        sequence: int,
        connected: bool,
        healthy: bool,
        reason: str,
        aircraft_indices: tuple[int, ...],
        received_s: float,
    ) -> None:
        if sequence <= self._latest_sequence:
            return
        self._latest_sequence = sequence
        self._latest_received_s = received_s
        self._active = ()
        self._active_sequence = -1
        if not connected or not healthy:
            self._latest_healthy = False
            self._latest_reason = reason or (
                "TRAFFIC_DISCONNECTED" if not connected else "UNHEALTHY_TRAFFIC"
            )
            self._purge()
            return
        if len(set(aircraft_indices)) != len(aircraft_indices):
            self._latest_healthy = False
            self._latest_reason = "DUPLICATE_TRAFFIC_IDENTITY"
            self._purge()
            return
        if self.own_aircraft_index in aircraft_indices:
            self._latest_healthy = False
            self._latest_reason = "OWN_AIRCRAFT_NOT_FILTERED"
            self._purge()
            return
        self._latest_healthy = True
        self._latest_reason = "INCOMPLETE_TRAFFIC_SNAPSHOT"
        self._expected[sequence] = aircraft_indices
        self._try_commit(sequence)
        self._purge()

    def _try_commit(self, sequence: int) -> None:
        if sequence != self._latest_sequence or sequence not in self._expected:
            return
        expected = self._expected[sequence]
        available = self._pending.get(sequence, {})
        if set(available) != set(expected):
            return
        self._active = tuple(available[index] for index in expected)
        self._active_sequence = sequence
        self._latest_reason = ""

    def _purge(self) -> None:
        self._pending = {
            sequence: value
            for sequence, value in self._pending.items()
            if sequence >= self._latest_sequence
        }
        self._expected = {
            sequence: value
            for sequence, value in self._expected.items()
            if sequence >= self._latest_sequence
        }

    def current(self, now_s: float) -> tuple[tuple[ObstacleRecord, ...], str | None]:
        if not self._latest_healthy or self._active_sequence != self._latest_sequence:
            return (), self._latest_reason
        assert self._latest_received_s is not None
        age_s = now_s - self._latest_received_s
        if age_s < 0.0:
            return (), "FUTURE_TRAFFIC"
        if age_s > self.freshness_s:
            return (), "STALE_TRAFFIC"
        return self._active, None
