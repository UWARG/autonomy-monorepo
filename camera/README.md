# OAK-D BasaltVIO integration

The camera module provides RGB/depth capture and optional local visual-inertial
pose estimates. Related task: [#193](https://github.com/UWARG/autonomy-monorepo/issues/193).
Implementation and validation tools: [PR #231](https://github.com/UWARG/autonomy-monorepo/pull/231).

**Software implementation is ready for hardware validation.** The automated
tests do not establish camera operation or real-world accuracy. Hardware
validation and team review remain required before closing the task.

## Setup

Use Ubuntu 24.04 / Python 3.12 for the prepared test workflow. The code requires
Python 3.10 or newer. A Linux wheel with Basalt support is required to run VIO;
the Windows DepthAI 3.10.0 wheel checked during development omitted BasaltVIO.

From the repository root:

```bash
cd camera
uv sync --locked --extra dev --extra oakd
uv run python -m tools.check_vio
uv run --extra dev --extra oakd pytest -q
uv run --extra dev ruff check .
```

The SDK check does not open the camera. If it reports missing Basalt support,
use a supported Linux environment or a Basalt-enabled SDK build; reinstalling
the same Windows wheel will not add the node.

The existing `oakd_test.py` and `arducam_test.py` files are manual hardware
scripts. Pytest collects `test_*.py` only, so CI never opens a camera by accident.
The project's PR CI runs both lint and the automated tests.

## Application API

```python
from src import OakD

camera = OakD(enable_vio=True)
try:
    if not camera.initialize_camera():
        raise RuntimeError("Camera initialization failed")
    # Call repeatedly in the application's loop:
    pose = camera.capture_pose()
    if pose is not None:
        print(pose.position_m, pose.orientation_xyzw)
finally:
    camera.close_camera()
```

`OakD()` keeps VIO disabled. `capture_pose()` returns the latest valid newly
queued estimate, or `None`; it does not return a cached pose. It drains at most
eight queue entries and discards intermediate estimates. It is a latest-pose
interface, not a lossless trajectory stream. Application consumers must handle
`None`, discontinuities, and estimator resets.

Position is in metres; quaternion order is x,y,z,w. Both remain in the SDK's
local frame. No vehicle-body, NED, GPS, ROS, or MAVLink conversion is provided.
SDK Buffer timestamps may be unset and are then represented as `None`.
`received_monotonic_s` is the time the host polled the queue, not exposure time,
end-to-end latency, or tracking confidence. RGB/depth and pose are exposed
separately and are not promised to represent the same capture instant.

VIO shares one pair of stereo streams with depth processing, using 640x400
images at 60 FPS and raw IMU data requested at 200 Hz. VIO-disabled depth uses
1280x800 inputs at 30 FPS. Check the effect of VIO's lower depth resolution and
host processing load on the team's application. RGB/depth polling is bounded
so an absent frame cannot indefinitely block the capture worker's shutdown.

## Validation tools

All commands below run from `camera/`. See the complete
[hardware test procedure](vio_hardware_validation.md) before testing.

```bash
# With an actual IMU-equipped OAK-D connected: require RGB, depth, and poses.
uv run python -m tools.check_vio --hardware

# Record a new trial; use a unique output name for every run.
uv run python -m tools.record_vio --output trials/stationary-01.csv --duration 60 --warmup 5 --camera-model "MODEL FROM LABEL" --notes "Stationary, fixed mount, lighting notes"

# Analyze the recording. No tolerance supplied means no pass claim.
uv run python -m tools.analyze_vio trials/stationary-01.csv --trial stationary --output trials/stationary-01.analysis.json
```

The recorder creates a CSV and `.metadata.json` manifest, including environment,
requested duration, notes, counts, completion state, and basic descriptive
metrics. Files are never overwritten. A recording with no valid poses for five
seconds fails (configurable with `--max-silence`). Ctrl+C preserves the partial
CSV and records `interrupted`, including during warmup. Failed or interrupted
trials are not successful measurements, even if they contain some samples.

The analyzer validates CSV data, averages stationary endpoint windows, compares
distance and principal rotation magnitudes with supplied ground truth, and
reports closed-path endpoint drift as a percentage of measured physical path
length. Reports use `not_assessed` until the necessary team-approved tolerances
are supplied. It handles quaternion sign equivalence. It does not infer ground
truth from the estimated trajectory or certify coordinate axes or flight safety.

## Coverage and remaining gate

Automated coverage includes pose validation, SDK point/quaternion accessors,
pipeline connections through an SDK double, disabled-VIO behavior, startup
failure, RGB/depth conversion, bounded shutdown, CSV/manifest recording,
interruption, silence timeout, and analysis against synthetic known geometry.
Synthetic fixtures are software test data, not camera experiment results.

Actual USB transport, device calibration and synchronization, estimator
initialization, tracking quality, resource use, and physical drift still need
the camera. Record those results in the test report in the linked procedure.

## Sources

- [Luxonis Basalt example](https://github.com/luxonis/depthai-core/blob/main/examples/python/VSLAM/basalt_vio.py)
- [Luxonis Basalt API](https://github.com/luxonis/depthai-core/blob/main/include/depthai/basalt/BasaltVIO.hpp)
- [Microsoft WSL USB setup](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
