"""
Unit tests for splitting a dropped drone's Missions among survivors.
"""

from __future__ import annotations

from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.survey_executive.redistribute import split_among
from tests.unit.survey_executive.factories import LAT0, LON0


def line_of_missions(count: int) -> list[Mission]:
    """
    Missions about 11 m apart heading north, yaws all different.
    """

    return [
        Mission(
            task_id=f"s-r0-x-{index:03d}",
            target=Coordinate(LAT0 + index * 1e-4, LON0, 30.0),
            yaw=0.1 * index,
        )
        for index in range(count)
    ]


def test_chunks_keep_order_and_every_mission_once() -> None:
    missions = line_of_missions(10)

    split = split_among(missions, {"a": None, "b": None, "c": None})

    chunks = sorted(split.values(), key=lambda chunk: missions.index(chunk[0]))
    assert [len(chunk) for chunk in chunks] == [4, 3, 3]
    assert [m for chunk in chunks for m in chunk] == missions


def test_each_chunk_goes_to_the_nearest_drone() -> None:
    missions = line_of_missions(6)
    # "a" waits beside the second half, "b" beside the first.
    survivors = {"a": missions[3].target, "b": missions[0].target}

    split = split_among(missions, survivors)

    assert split == {"a": missions[3:], "b": missions[:3]}


def test_drones_with_unknown_place_come_last() -> None:
    missions = line_of_missions(4)

    split = split_among(missions, {"x": None, "y": missions[0].target})

    assert split == {"y": missions[:2], "x": missions[2:]}


def test_one_survivor_takes_everything() -> None:
    missions = line_of_missions(5)

    assert split_among(missions, {"a": None}) == {"a": missions}


def test_more_survivors_than_missions() -> None:
    missions = line_of_missions(2)
    survivors = {"a": missions[1].target, "b": missions[0].target, "c": None}

    assert split_among(missions, survivors) == {
        "a": [missions[1]],
        "b": [missions[0]],
    }


def test_nothing_to_split() -> None:
    assert split_among([], {"a": None}) == {}
    assert split_among(line_of_missions(3), {}) == {}


def test_missions_are_not_changed() -> None:
    missions = line_of_missions(3)
    before = list(missions)

    split = split_among(missions, {"a": None, "b": None})

    assert missions == before
    assert all(m in before for chunk in split.values() for m in chunk)
