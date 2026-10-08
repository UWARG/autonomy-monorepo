"""Fixtures shared by the unit and integration tests."""

from __future__ import annotations

import gc
import logging
from collections.abc import Iterator

import pytest

from tests.fakes import FakeClock, LocationCommandFactory, make_location_command


@pytest.fixture
def fake_clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def location_command() -> LocationCommandFactory:
    return make_location_command


class _ErrorCollector(logging.Handler):
    def __init__(self) -> None:
        super().__init__(logging.ERROR)
        self.messages: list[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.messages.append(record.getMessage())


@pytest.fixture(autouse=True)
def _fail_on_asyncio_errors() -> Iterator[None]:
    """Fail tests that leave errors asyncio only logs, e.g. "Task exception was
    never retrieved", which would otherwise pass silently."""
    collector = _ErrorCollector()
    asyncio_logger = logging.getLogger("asyncio")
    asyncio_logger.addHandler(collector)
    try:
        yield
        gc.collect()  # asyncio reports unretrieved task errors on garbage collection
    finally:
        asyncio_logger.removeHandler(collector)
    if collector.messages:
        pytest.fail(f"asyncio logged errors: {collector.messages}")
