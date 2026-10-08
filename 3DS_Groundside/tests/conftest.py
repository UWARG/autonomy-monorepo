"""Fixtures shared by the unit and integration tests."""

import pytest

from tests.fakes import FakeClock, LocationCommandFactory, make_location_command


@pytest.fixture
def fake_clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def location_command() -> LocationCommandFactory:
    return make_location_command
