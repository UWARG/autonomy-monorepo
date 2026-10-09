"""Unit tests for the fleet launcher's helpers (no processes started)."""

from __future__ import annotations

import math
from typing import Any

import pytest

from src.mock_fleet import FleetConfig, drone_ids, parse_args, start_positions


def test_drone_ids_are_unique_and_zero_padded() -> None:
    assert drone_ids(3) == ["drone-01", "drone-02", "drone-03"]


def test_drone_ids_widen_padding_so_they_still_sort() -> None:
    ids = drone_ids(120)

    assert ids[0] == "drone-001"
    assert ids[-1] == "drone-120"
    assert ids == sorted(ids)
    assert len(set(ids)) == 120


def test_start_positions_are_spread_along_x_at_altitude() -> None:
    assert start_positions(3, spacing=5.0, altitude=15.0) == [
        (0.0, 0.0, 15.0),
        (5.0, 0.0, 15.0),
        (10.0, 0.0, 15.0),
    ]


def test_parse_args_defaults() -> None:
    assert parse_args([]) == FleetConfig(num_drones=3, url="ws://127.0.0.1:8765")


def test_parse_args_reads_every_option() -> None:
    config = parse_args(
        [
            "--num-drones", "7",
            "--url", "ws://groundside:9000",
            "--speed", "2.5",
            "--spacing", "0",
            "--altitude", "30",
            "--report-period", "0.5",
            "--log-level", "DEBUG",
        ]
    )  # fmt: skip

    assert config == FleetConfig(
        num_drones=7,
        url="ws://groundside:9000",
        speed=2.5,
        spacing=0.0,
        altitude=30.0,
        report_period=0.5,
        log_level="DEBUG",
    )


@pytest.mark.parametrize(
    "argv",
    [
        ["--num-drones", "0"],
        ["--speed", "0"],
        ["--speed", "nan"],
        ["--spacing", "nan"],
        ["--altitude", "inf"],
        ["--report-period", "-1"],
        ["--log-level", "LOUD"],
        ["--url", "ws//missing-colon"],
    ],
)
def test_parse_args_rejects_invalid_settings(
    argv: list[str], capsys: pytest.CaptureFixture[str]
) -> None:
    with pytest.raises(SystemExit):
        parse_args(argv)

    assert "error:" in capsys.readouterr().err


def test_validate_accepts_valid_config() -> None:
    # The baseline the rejection cases below start from must itself be valid.
    FleetConfig(num_drones=2, url="ws://x").validate()
    FleetConfig(num_drones=2, url="wss://groundside.example:443/path").validate()


@pytest.mark.parametrize(
    "overrides",
    [
        {"num_drones": 0},
        {"speed": math.nan},
        {"spacing": math.nan},
        {"altitude": math.inf},
        {"url": "ws//missing-colon"},
        {"url": "http://wrong-scheme"},
    ],
)
def test_validate_rejects_invalid_config(overrides: dict[str, Any]) -> None:
    settings: dict[str, Any] = {"num_drones": 2, "url": "ws://x"}
    settings.update(overrides)

    with pytest.raises(ValueError):
        FleetConfig(**settings).validate()
