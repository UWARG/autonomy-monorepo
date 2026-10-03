from dataclasses import FrozenInstanceError

import pytest

from orthomosaic.models import CameraSpec, ScanRequest


def test_scan_request_defaults():
    request = ScanRequest(
        boundary=[],
        camera=CameraSpec(1280, 720, 1.0, 1.0),
        target_gsd_m=0.02,
    )
    assert request.forward_overlap == 0.75
    assert request.side_overlap == 0.65
    assert request.start is None
    assert request.turn_penalty_m is None
    assert request.angle_step_deg == 5.0


def test_scan_request_frozen():
    request = ScanRequest(
        boundary=[],
        camera=CameraSpec(1280, 720, 1.0, 1.0),
        target_gsd_m=0.02,
    )
    with pytest.raises(FrozenInstanceError):
        request.forward_overlap = 0.5