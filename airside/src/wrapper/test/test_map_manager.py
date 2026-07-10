import json
from datetime import datetime

import pytest

from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.msg import Target as TargetMsg

from utils.src.enums import Colours
from utils.src.types import Coordinate, Target

from wrapper.map_manager_node import TargetLog, target_from_msg


def test_target_from_msg():
    msg = TargetMsg(
        colour="RED", location=CoordinateMsg(lat=1.0, lon=2.0, alt=3.0)
    )
    target = target_from_msg(msg)
    assert target.colour is Colours.RED
    assert target.location == Coordinate(lat=1.0, lon=2.0, alt=3.0)


def test_target_from_msg_rejects_unknown_colour():
    msg = TargetMsg(
        colour="MAGENTA", location=CoordinateMsg(lat=0.0, lon=0.0, alt=0.0)
    )
    with pytest.raises(KeyError):
        target_from_msg(msg)


def test_target_log_wipes_previous_run(tmp_path):
    working = tmp_path / TargetLog.WORKING_FILENAME
    working.write_text("stale line from previous run\n")

    log = TargetLog(tmp_path)
    assert log.working_file.read_text() == ""


def test_target_log_append_and_archive(tmp_path):
    log = TargetLog(tmp_path)
    target = Target(colour=Colours.BLUE, location=Coordinate(1.0, 2.0, 3.0))
    log.append(target, stamp="2026-07-09T12:00:00")

    record = json.loads(log.working_file.read_text().splitlines()[0])
    assert record == {
        "stamp": "2026-07-09T12:00:00",
        "colour": "BLUE",
        "lat": 1.0,
        "lon": 2.0,
        "alt": 3.0,
    }

    archive = log.archive(datetime(2026, 7, 9, 12, 30, 0))
    assert archive.name == "targets_2026-07-09T12-30-00.jsonl"
    assert archive.read_text() == log.working_file.read_text()
