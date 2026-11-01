"""Deterministic mock traffic and independent true-separation checks."""

from __future__ import annotations

import json
import math

import pytest

from live_backup.evaluate_moving_truth import evaluate
from live_backup.mock_aeac_server import FixtureState


def fixture(mode: str, tmp_path) -> FixtureState:
    state = FixtureState(mode=mode, summary=tmp_path / "summary.json")
    state.origin = (43.0, -80.0)
    state.movement_started_s = 100.0
    return state


def test_moving_aircraft_position_matches_reported_speed_and_heading(tmp_path) -> None:
    state = fixture("moving", tmp_path)
    first = state._other_aircraft(100.0)[0]
    second = state._other_aircraft(101.0)[0]
    assert first["position"]["speed"] == pytest.approx(1.2)
    assert first["position"]["direction"] == pytest.approx(90.0)
    assert second["position"]["lon"] > first["position"]["lon"]
    assert second["position"]["lat"] == pytest.approx(first["position"]["lat"])
    assert second["position"]["altitude"] == pytest.approx(30.0)


def test_multi_and_turn_are_coherent(tmp_path) -> None:
    multi = fixture("multi", tmp_path)
    assert {aircraft["aircraftIndex"] for aircraft in multi._other_aircraft(104.0)} == {2, 3}
    turning = fixture("turn", tmp_path)
    before = turning._other_aircraft(106.0)[0]
    after = turning._other_aircraft(107.0)[0]
    assert after["position"]["lon"] == pytest.approx(before["position"]["lon"])
    assert after["position"]["lat"] > before["position"]["lat"]
    assert after["position"]["direction"] == pytest.approx(0.0)


def test_blocked_target_occupies_goal(tmp_path) -> None:
    blocked = fixture("blocked", tmp_path)._other_aircraft(100.0)[0]
    assert blocked["position"]["speed"] == 0.0
    assert blocked["position"]["lat"] > 43.0


def test_true_separation_evaluator_rejects_breach(tmp_path) -> None:
    own = tmp_path / "own.jsonl"
    truth = tmp_path / "truth.jsonl"
    sample = fixture("blocked", tmp_path)._other_aircraft(100.0)
    target = sample[0]["position"]
    own.write_text(
        json.dumps({"armed": True, "mode": "GUIDED", "monotonic_s": 100.05,
                    "latitude_deg": target["lat"], "longitude_deg": target["lon"]}) + "\n",
        encoding="utf-8",
    )
    truth.write_text(
        "\n".join(json.dumps({"monotonic_s": stamp, "aircraft": sample})
                  for stamp in (100.0, 100.1)) + "\n",
        encoding="utf-8",
    )
    result = evaluate(own, truth, 3.0)
    assert result["minimum_true_clearance_m"] == -5.0
    assert result["verdict"] == "FAIL"


def test_true_separation_evaluator_accepts_margin(tmp_path) -> None:
    own = tmp_path / "own.jsonl"
    truth = tmp_path / "truth.jsonl"
    target = fixture("blocked", tmp_path)._other_aircraft(100.0)[0]
    own.write_text(
        json.dumps({
            "armed": True, "mode": "GUIDED", "monotonic_s": 100.05,
            "latitude_deg": target["position"]["lat"] + math.degrees(10 / 6_371_000),
            "longitude_deg": target["position"]["lon"],
        }) + "\n",
        encoding="utf-8",
    )
    truth.write_text(
        "\n".join(json.dumps({"monotonic_s": stamp, "aircraft": [target]})
                  for stamp in (100.0, 100.1)) + "\n",
        encoding="utf-8",
    )
    result = evaluate(own, truth, 3.0)
    assert result["minimum_true_clearance_m"] == pytest.approx(5.0, abs=0.001)
    assert result["verdict"] == "PASS"
