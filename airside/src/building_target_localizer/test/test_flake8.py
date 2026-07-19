"""ROS ament flake8 integration."""

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Require the package to pass the ROS Python style profile."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, "Found style errors:\n" + "\n".join(errors)
