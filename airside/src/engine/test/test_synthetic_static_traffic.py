from __future__ import annotations

import math

import pytest
from engine.synthetic_static_traffic import (
    EARTH_RADIUS_M,
    SyntheticStaticTrafficConfig,
    offset_coordinate,
)


def test_default_matches_static_sitl_obstacle() -> None:
    config = SyntheticStaticTrafficConfig()
    latitude, longitude = offset_coordinate(
        43.0,
        -80.0,
        config.east_offset_m,
        config.north_offset_m,
    )

    north_m = math.radians(latitude - 43.0) * EARTH_RADIUS_M
    assert north_m == pytest.approx(20.0)
    assert longitude == pytest.approx(-80.0)
    assert config.altitude_agl_m == 15.0
    assert config.horizontal_keepaway_m == 5.0
    assert config.vertical_keepaway_m == 5.0


def test_coordinate_offsets_preserve_signs() -> None:
    unchanged = offset_coordinate(43.0, -80.0, 0.0, 0.0)
    northeast = offset_coordinate(43.0, -80.0, 10.0, 10.0)

    assert unchanged == (43.0, -80.0)
    assert northeast[0] > 43.0
    assert northeast[1] > -80.0


@pytest.mark.parametrize(
    "override",
    [
        {"horizontal_keepaway_m": 0.0},
        {"vertical_keepaway_m": -1.0},
        {"publish_rate_hz": 0.0},
        {"north_offset_m": math.nan},
    ],
)
def test_invalid_configuration_is_rejected(override: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        SyntheticStaticTrafficConfig(**override)


def test_pinned_coordinate_is_optional() -> None:
    assert SyntheticStaticTrafficConfig().latitude_deg is None

    config = SyntheticStaticTrafficConfig(
        latitude_deg=43.4339558, longitude_deg=-80.5777818
    )

    assert (config.latitude_deg, config.longitude_deg) == (43.4339558, -80.5777818)


@pytest.mark.parametrize(
    "override",
    [
        {"latitude_deg": 43.0},
        {"longitude_deg": -80.0},
        {"latitude_deg": 91.0, "longitude_deg": -80.0},
        {"latitude_deg": 43.0, "longitude_deg": math.inf},
    ],
)
def test_invalid_pinned_coordinate_is_rejected(override: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        SyntheticStaticTrafficConfig(**override)
