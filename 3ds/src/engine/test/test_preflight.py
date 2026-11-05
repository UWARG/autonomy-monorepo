"""Hardware-free regression tests for preflight decisions."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engine.preflight import (
    ERROR,
    OK,
    PREARM_CHECK,
    STALE,
    WARN,
    Check,
    Stream,
    image_check,
    overall,
    prearm_check,
)


def test_stream_startup_timeout_and_recovery():
    stream = Stream(3.0)
    assert stream.check(0).level == STALE
    stream.tick(1)
    assert stream.check(4).level == OK
    assert stream.check(4.001).level == STALE
    stream.tick(5)
    assert stream.check(5).level == OK


def test_minimum_rate_and_recovery():
    stream = Stream(3.0, 6.0)
    stream.tick(0)
    assert stream.check(0).level == WARN
    stream.tick(1)
    assert stream.check(1).level == WARN
    for i in range(20):
        stream.tick(6 + i * 0.1)
    assert stream.check(7.9).level == OK
    assert stream.check(9.9).level == WARN
    assert stream.check(11).level == STALE


@pytest.mark.parametrize(
    "timeout,min_hz", [(0, 0), (-1, 0), (float("nan"), 0), (1, -1), (1, float("inf"))]
)
def test_invalid_stream_configuration(timeout, min_hz):
    with pytest.raises(ValueError):
        Stream(timeout, min_hz)


@pytest.mark.parametrize(
    "present,enabled,health,armed,level",
    [
        (0, 0, 0, False, STALE),
        (PREARM_CHECK, 0, PREARM_CHECK, False, STALE),
        (PREARM_CHECK, PREARM_CHECK, 0, False, ERROR),
        (PREARM_CHECK, PREARM_CHECK, PREARM_CHECK, False, OK),
        (PREARM_CHECK, PREARM_CHECK, PREARM_CHECK, True, ERROR),
        (0, PREARM_CHECK, PREARM_CHECK, False, STALE),
    ],
)
def test_prearm_needs_positive_evidence(present, enabled, health, armed, level):
    assert prearm_check(present, enabled, health, armed).level == level


@pytest.mark.parametrize(
    "encoding,channels",
    [("mono8", 1), ("rgb8", 3), ("bgr8", 3), ("rgba8", 4), ("bgra8", 4)],
)
def test_textured_image_and_padded_rows(encoding, channels):
    rows = np.full((64, 64 * channels + 4), 255, dtype=np.uint8)
    pixels = rows[:, : 64 * channels].reshape(64, 64, channels)
    pixels[::2, :, : min(channels, 3)] = 10
    assert (
        image_check(rows.tobytes(), 64, 64, rows.shape[1], encoding, 5, 2).level == OK
    )


@pytest.mark.parametrize(
    "color", [(0, 0, 0), (255, 255, 255), (255, 0, 0), (128, 128, 128)]
)
def test_blank_and_solid_color_frames_fail(color):
    image = np.tile(np.array(color, dtype=np.uint8), (64, 64, 1))
    assert image_check(image.tobytes(), 64, 64, 192, "rgb8", 5, 2).level == ERROR


def test_padding_and_alpha_do_not_make_blank_image_healthy():
    rows = np.zeros((64, 64 * 4 + 4), dtype=np.uint8)
    rows[:, -4:] = 255
    rows[:, :256].reshape(64, 64, 4)[:, :, 3] = 255
    assert image_check(rows.tobytes(), 64, 64, 260, "rgba8", 5, 2).level == ERROR


@pytest.mark.parametrize(
    "data,width,height,step,encoding",
    [
        (b"", 0, 0, 0, "rgb8"),
        (b"", 64, 64, 192, "rgb8"),
        (bytes(100), 10, 10, 1, "rgb8"),
        (bytes(100), 10, 10, 10, "mono16"),
    ],
)
def test_bad_frames_fail(data, width, height, step, encoding):
    assert image_check(data, width, height, step, encoding, 5, 2).level == ERROR


def test_readiness_requires_every_check_to_pass():
    assert overall({}).level == STALE
    assert overall({"camera": Check(OK, "usable")}).level == OK
    for level in (WARN, ERROR, STALE):
        result = overall(
            {"camera": Check(OK, "usable"), "groundstation": Check(level, "lost")}
        )
        assert result.level != OK
        assert "groundstation: lost" in result.message
