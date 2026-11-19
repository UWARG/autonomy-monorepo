# VIO hardware validation procedure and report

Status: **pending physical camera tests**. Do not fill measured results with
synthetic unit-test values. The task requires integration AND effectiveness
testing; passing software checks alone does not complete the hardware gate.

## Before the test session

1. Arrange an IMU-equipped calibrated OAK-D, USB 3 data cable, suitable Linux
   host, rigid camera mount, tape measure, and marked rotation reference.
2. Agree with the lead on acceptable stationary displacement, measured-translation
   distance error, rotation angle error and unwanted translation, closed-path
   drift, receipt gaps, and runtime load. Write these limits below before trials.
3. Use a fixed, well-lit textured scene. Start with handheld or fixed-mount bench
   tests with propulsion disabled; flight validation is a later team decision.
4. Record the actual camera model, IMU model if known, SDK version, host OS,
   USB connection, lighting, movement speed, and code revision. Camera calibration
   must include usable stereo intrinsics/extrinsics and camera-to-IMU alignment.

```bash
cd ~/autonomy-monorepo
git status --short
git switch basalt-vio-integration
git pull --ff-only origin basalt-vio-integration
git rev-parse HEAD
cd camera
uv sync --locked --extra dev --extra oakd
uv run --extra dev --extra oakd pytest -q
uv run --extra dev ruff check .
uv run python -m tools.check_vio
```

If `git status` reports personal edits, preserve them before switching/updating.
Do not reset them. Verify camera label/model; some OAK variants do not contain
the IMU required by this integration.

## WSL USB attachment (only if using Windows/WSL)

Follow [Microsoft's procedure](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).
Keep an Ubuntu terminal open. In Windows PowerShell, install usbipd if absent:

```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
```

Choose the OAK camera's actual BUSID from that list. In an administrator
PowerShell window, run `usbipd bind --busid BUSID`, replacing BUSID with that
device's value. Then in PowerShell run `usbipd attach --wsl --busid BUSID`.
In Ubuntu, `lsusb` should show the camera (install `usbutils` if needed).
If the device re-enumerates during boot, recheck `usbipd list` and its attachment.
If WSL transport remains unreliable, use a native Linux test host.

For a Linux USB permission error, apply the
[Luxonis device setup guidance](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/).
The commonly used Luxonis USB rule is:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect/reattach the camera afterward. If udev is not running in the chosen
WSL configuration, use a native Linux host or resolve that host setup first.
Do not treat a USB failure as evidence of estimator accuracy or inaccuracy.

## Hardware startup gate

Run the official Luxonis Basalt example on the same camera/SDK if necessary
to isolate SDK/calibration/device problems from the integration. Then run:

```bash
uv run python -m tools.check_vio --hardware --timeout 15
```

This requires multiple RGB/depth frames and finite pose estimates from the
combined pipeline. It establishes output availability only. Inspect logs for
IMU/calibration/synchronization errors, check image quality and depth units,
and check for pose resets. Physically move the camera along marked axes to
identify local-frame conventions. No body-frame transformation is assumed.

## Trials: at least three repeats of each

Hold still during warmup and for at least two seconds at each recording
endpoint. The analyzer defaults to one-second endpoint windows. Begin motion
only after `RECORDING` appears. Finish motion before the recording ends.

Record with a unique name (the example name is for the stationary trial):

```bash
uv run python -m tools.record_vio --output trials/stationary-01.csv --duration 60 --warmup 5 --camera-model "ACTUAL MODEL" --notes "Mount, lighting, host, trial details"
```

The `.metadata.json` status must be `complete`; `failed` and `interrupted` trials
remain diagnostic artifacts and do not count as passing repeats. Preserve SDK
logs, the raw CSV, metadata, analyzer JSON, and independent measurements.

### Stationary

Fix the camera for 60 seconds. Analyze maximum displacement from the initial
endpoint mean, endpoint displacement, rotation, and maximum receipt gap:

```bash
uv run python -m tools.analyze_vio trials/stationary-01.csv --trial stationary --output trials/stationary-01.analysis.json
```

### Translation

Move between physically measured endpoints at a moderate speed. Repeat along
multiple marked axes and directions. The example assumes the independently
measured distance is exactly 2.0 m; replace it with your measurement:

```bash
uv run python -m tools.analyze_vio trials/translation-01.csv --trial translation --expected-distance-m 2.0 --output trials/translation-01.analysis.json
```

Distance magnitude alone does not check axis signs or body-frame alignment.
Document observed axes and any unexpected direction changes separately.

### Rotation

Rotate around approximately the camera optical centre through an independently
marked 90-degree turn, holding both endpoint orientations. Report angle error
and unwanted translation; document the actual pivot and inevitable translation.
The analyzer measures the principal endpoint angle (0 to 180 degrees), not
accumulated full revolutions. Test a 360-degree return as a return-to-start trial,
not as `--expected-angle-deg 360`.

```bash
uv run python -m tools.analyze_vio trials/rotation-01.csv --trial rotation --expected-angle-deg 90 --output trials/rotation-01.analysis.json
```

### Closed path / return to start

Traverse a measured physical path and return to the same marked position AND
orientation. The example is an independently measured 8 m path. Do not use
the VIO-estimated path length as ground truth:

```bash
uv run python -m tools.analyze_vio trials/closed-loop-01.csv --trial closed-loop --path-length-m 8.0 --output trials/closed-loop-01.analysis.json
```

### Acceptance and adverse conditions

Add the agreed `--position-tolerance-m`, `--max-gap-s`, and (for rotation)
`--angle-tolerance-deg` values to the analysis command. No defaults silently
declare accuracy acceptable. Missing required tolerances produce `not_assessed`;
a failed supplied criterion produces a nonzero exit code when the required
trial criteria are present. Passing criteria apply only to that recorded trial.

Inspect host CPU/memory load with VIO disabled and enabled. Repeat selected
trials with lower texture, lower light, and faster turns. Check dropped/missing
poses, discontinuities, recovery, depth quality, and shutdown/reconnect behavior.
Host receipt gaps are not exposure-to-host latency or tracking confidence.

## Test report to complete at the session

Date / operator / code revision: **PENDING**

Camera / IMU / calibration record / cable / USB mode: **PENDING**

Host / OS / DepthAI / lighting / motion conditions: **PENDING**

Lead-approved position, angle, gap, and load limits: **PENDING**

| Trial and repeat | Ground truth | CSV + metadata + JSON + logs | Error / gap / load | Outcome |
|---|---|---|---|---|
| Stationary 1–3 | Fixed pose, 60 s each | Pending | Not measured | Pending |
| Translation 1–3 per axis | Measured distance and direction | Pending | Not measured | Pending |
| Rotation 1–3 | Measured angle and pivot | Pending | Not measured | Pending |
| Closed path 1–3 | Measured physical path; same endpoint | Pending | Not measured | Pending |
| Adverse conditions / recovery | Documented conditions | Pending | Not measured | Pending |

Attach the completed report to PR #231. Resolve findings, get lead review, and
only then decide whether issue #193's effectiveness requirement is satisfied.
