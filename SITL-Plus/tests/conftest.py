"""Pytest fixtures for SITL-Plus tests."""

from pathlib import Path

import pybullet as p
import pybullet_data
import pytest

import iris
import state
from camera import Camera
from range_finder import Range_Finder

_TEST_MODELS = Path(__file__).resolve().parent / "assets"


@pytest.fixture(scope="session", name="_bullet_connect")
def fixture_bullet_connect():
    """Initialize a headless PyBullet session for tests."""
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.80665)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    state.dir_path = _TEST_MODELS
    iris.Iris()


@pytest.fixture(name="camera_obj")
def fixture_camera_obj():
    """Return a downward-facing test camera."""
    return Camera(attached_to_object=1, port=6000, direction=[0, 0, -1], depth_map=True)


@pytest.fixture(name="range_finder_obj")
def fixture_range_finder_obj():
    """Return a downward-facing test range finder."""
    return Range_Finder(port=6004, direction=[0, 0, -1], dist=100)


@pytest.fixture(name="iris_obj")
def fixture_iris_obj():
    """Return an Iris vehicle loaded into the current PyBullet session."""
    state.robot_id = p.loadURDF(str(state.dir_path / "iris/iris.urdf"), [0, 0, 0.2])
    return iris.Iris()
