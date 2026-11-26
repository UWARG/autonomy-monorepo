from pathlib import Path

from check_field_config import check_config


def test_missing_settings_fail_closed_without_disclosing_token(tmp_path: Path) -> None:
    route = tmp_path / "route.yaml"
    route.write_text(
        "waypoints:\n  - {lat: 43.0, lon: -80.0, alt: 10}\n"
        "  - {lat: 43.001, lon: -80.0, alt: 10}\n"
        "  - {lat: 43.0, lon: -79.999, alt: 10}\n",
        encoding="utf-8",
    )
    problems = check_config(
        {"AEAC_CONNECTION_TOKEN": "secret-value"}, route, "props-off"
    )
    assert problems
    assert "secret-value" not in str(problems)
    assert "AEAC_OWN_AIRCRAFT_INDEX missing or invalid" in problems


def test_valid_local_geometry_is_only_a_config_check(tmp_path: Path) -> None:
    route = tmp_path / "route.yaml"
    route.write_text(
        "waypoints:\n  - {lat: 43.0, lon: -80.0, alt: 10}\n"
        "  - {lat: 43.001, lon: -80.0, alt: 10}\n"
        "  - {lat: 43.0, lon: -79.999, alt: 10}\n",
        encoding="utf-8",
    )
    settings = {
        "AEAC_CONNECTION_TOKEN": "secret-value",
        "AEAC_UAV_ID": "TEST-UAV",
        "AEAC_WEBSOCKET_URL": "wss://example.invalid",
        "AEAC_OWN_AIRCRAFT_INDEX": "2",
        "FLIGHT_BOUNDARY_WGS84": "[[42.99,-80.01],[42.99,-79.98],[43.02,-79.98],[43.02,-80.01]]",
        "USE_AEAC_TELEMETRY": "true",
        "USE_ENGINE": "false",
    }
    assert check_config(settings, route, "props-off") == []
    assert check_config(settings, route, "flight") == [
        "USE_ENGINE must be true for flight"
    ]
