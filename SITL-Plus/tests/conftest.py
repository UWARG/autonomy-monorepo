"""Pytest fixtures for SITL-Plus tests."""

from pathlib import Path

import pytest

import iris
import state
from environment import Environment
from camera import Camera
from range_finder import Range_Finder

_TEST_MODELS = Path(__file__).resolve().parent / "assets"


@pytest.fixture(scope="session", name="_world")
def fixture_world():
    """Initialize one headless world for all test vehicles."""
    environment = Environment()
    state.dir_path = _TEST_MODELS
    yield environment
    environment.close()


@pytest.fixture(scope="session", name="_bullet_connect")
def fixture_bullet_connect(_world):
    """Return a body in the shared test world."""
    return iris.Iris()


@pytest.fixture(name="camera_obj")
def fixture_camera_obj(_bullet_connect):
    """Return a downward-facing test camera."""
    return Camera(
        attached_to_object=_bullet_connect.robot_id,
        port=6000,
        direction=[0, 0, -1],
        depth_map=True,
    )


@pytest.fixture(name="range_finder_obj")
def fixture_range_finder_obj(_bullet_connect):
    """Return a downward-facing test range finder."""
    return Range_Finder(
        attached_to_object=_bullet_connect.robot_id,
        port=6004,
        direction=[0, 0, -1],
        dist=100,
    )


@pytest.fixture(name="iris_obj")
def fixture_iris_obj(_bullet_connect):
    """Return an Iris vehicle loaded into the current PyBullet session."""
    return iris.Iris()
