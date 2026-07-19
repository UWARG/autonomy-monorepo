"""ROS ament docstring-lint integration."""
import pytest
from ament_pep257.main import main

@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Require public package APIs to carry useful docstrings."""
    assert main(argv=["building_target_localizer", "test"]) == 0
