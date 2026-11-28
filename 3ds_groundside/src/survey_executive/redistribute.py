"""
Splits a dropped drone's unflown Missions among the drones still flying.
"""

from __future__ import annotations

import math

from utils.src.coordinates import coordinate_to_enu
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission


def split_among(
    missions: list[Mission], survivors: dict[str, Coordinate | None]
) -> dict[str, list[Mission]]:
    """
    Cuts ``missions`` into one in-order chunk per survivor and gives each
    chunk to the survivor nearest its first Mission.

    ``survivors`` maps each drone ID to where it will be when it starts the
    chunk (the end of its queue, else its position), or None if unknown.
    Missions keep their task IDs, order and yaw; a chunk is never reversed,
    since each yaw faces the planned direction of flight. Drones with no
    known place come last, in the order given. Returns only the drones that
    got a chunk.
    """

    if not missions or not survivors:
        return {}
    chunks = _even_chunks(missions, len(survivors))
    order = {drone_id: rank for rank, drone_id in enumerate(survivors)}
    pairs = sorted(
        (
            _distance_m(place, chunk[0].target),
            chunk_index,
            order[drone_id],
            drone_id,
        )
        for chunk_index, chunk in enumerate(chunks)
        for drone_id, place in survivors.items()
    )
    # Greedy nearest pairing: good enough for a handful of drones, and
    # deterministic, which the tests rely on.
    assigned: dict[str, list[Mission]] = {}
    used_chunks: set[int] = set()
    for _, chunk_index, _, drone_id in pairs:
        if chunk_index in used_chunks or drone_id in assigned:
            continue
        assigned[drone_id] = list(chunks[chunk_index])
        used_chunks.add(chunk_index)
    return assigned


def _even_chunks(missions: list[Mission], count: int) -> list[list[Mission]]:
    """
    Splits into at most ``count`` contiguous chunks whose sizes differ by at
    most one, earlier chunks taking the extra Mission.
    """

    count = min(count, len(missions))
    size, extra = divmod(len(missions), count)
    chunks: list[list[Mission]] = []
    start = 0
    for index in range(count):
        end = start + size + (1 if index < extra else 0)
        chunks.append(missions[start:end])
        start = end
    return chunks


def _distance_m(place: Coordinate | None, target: Coordinate) -> float:
    """
    Horizontal distance in metres; infinite when the place is unknown.
    """

    if place is None:
        return math.inf
    east, north = coordinate_to_enu(place, target)
    return math.hypot(east, north)
