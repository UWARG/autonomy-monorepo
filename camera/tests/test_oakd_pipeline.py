"""Check pipeline wiring and lifecycle using a recording SDK double."""

import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from src.oakd import OakD


@pytest.fixture
def sdk(monkeypatch):
    types = SimpleNamespace(
        **{name: object() for name in ("Camera", "StereoDepth", "IMU", "BasaltVIO")}
    )
    cameras = [Mock() for _ in range(3)]
    stereo, imu, vio = Mock(), Mock(), Mock()
    for node in [*cameras, stereo]:
        node.build.return_value = node
    outputs = [camera.requestOutput.return_value for camera in cameras]
    for output in [*outputs, stereo.depth, vio.transform]:
        output.createOutputQueue.return_value.tryGet.return_value = None
    pipeline = Mock()
    pending_cameras = iter(cameras)

    def create(kind):
        if kind is types.Camera:
            return next(pending_cameras)
        return {types.StereoDepth: stereo, types.IMU: imu, types.BasaltVIO: vio}[kind]

    pipeline.create.side_effect = create
    dai = SimpleNamespace(
        Pipeline=Mock(return_value=pipeline),
        node=types,
        CameraBoardSocket=SimpleNamespace(CAM_B="B", CAM_C="C"),
        ImgFrame=SimpleNamespace(Type=SimpleNamespace(BGR888p="BGR")),
        IMUSensor=SimpleNamespace(ACCELEROMETER_RAW="accel", GYROSCOPE_RAW="gyro"),
    )
    monkeypatch.setitem(sys.modules, "depthai", dai)
    return SimpleNamespace(
        dai=dai,
        cameras=cameras,
        outputs=outputs,
        pipeline=pipeline,
        stereo=stereo,
        imu=imu,
        vio=vio,
    )


def test_vio_wiring_uses_one_pipeline_and_shared_stereo_streams(sdk):
    camera = OakD(enable_vio=True)
    assert camera.initialize_camera()
    sdk.dai.Pipeline.assert_called_once()
    for node, socket in zip(sdk.cameras[1:], ("B", "C")):
        node.build.assert_called_once_with(socket, sensorFps=60)
        node.requestOutput.assert_called_once_with((640, 400))
    sdk.stereo.build.assert_called_once_with(*sdk.outputs[1:])
    sdk.outputs[1].link.assert_called_once_with(sdk.vio.left)
    sdk.outputs[2].link.assert_called_once_with(sdk.vio.right)
    sdk.imu.enableIMUSensor.assert_called_once_with(["accel", "gyro"], 200)
    sdk.imu.out.link.assert_called_once_with(sdk.vio.imu)
    sdk.vio.setImuUpdateRate.assert_called_once_with(200)
    sdk.vio.transform.createOutputQueue.assert_called_once_with(
        maxSize=8, blocking=False
    )
    sdk.pipeline.start.assert_called_once()
    camera.close_camera()


def test_default_mode_does_not_create_vio_or_imu(sdk):
    camera = OakD()
    assert camera.initialize_camera()
    created = [call.args[0] for call in sdk.pipeline.create.call_args_list]
    assert sdk.dai.node.BasaltVIO not in created
    assert sdk.dai.node.IMU not in created
    for node in sdk.cameras[1:]:
        node.requestOutput.assert_called_once_with((1280, 800))
    assert camera.capture_pose() is None
    camera.close_camera()


def test_unsupported_sdk_fails_before_device_open(sdk):
    del sdk.dai.node.BasaltVIO
    camera = OakD(enable_vio=True)
    assert not camera.initialize_camera()
    sdk.dai.Pipeline.assert_not_called()


def test_partial_initialization_failure_releases_pipeline(sdk):
    sdk.pipeline.start.side_effect = RuntimeError("start failed")
    camera = OakD(enable_vio=True)
    assert not camera.initialize_camera()
    sdk.pipeline.stop.assert_called_once()
    assert camera._pipeline is camera._vio_queue is camera._rgb_queue is None


def test_frame_capture_preserves_rgb_depth_and_metres(sdk):
    camera = OakD()
    assert camera.initialize_camera()
    rgb = np.zeros((5, 5, 3), dtype=np.uint8)
    depth = np.full((5, 5), 2000, dtype=np.uint16)
    sdk.outputs[0].createOutputQueue.return_value.tryGet.return_value = SimpleNamespace(
        getCvFrame=lambda: rgb
    )
    sdk.stereo.depth.createOutputQueue.return_value.tryGet.return_value = (
        SimpleNamespace(getCvFrame=lambda: depth)
    )
    frame = camera.capture_frame()
    assert frame is not None
    assert frame.rgb is rgb
    assert frame.depth is depth
    assert frame.centre_depth == 2.0
    camera.close_camera()


def test_stop_unblocks_worker_waiting_for_missing_frames(sdk):
    waiting = threading.Event()

    def empty_queue():
        waiting.set()
        return None

    sdk.outputs[0].createOutputQueue.return_value.tryGet.side_effect = empty_queue
    camera = OakD()
    assert camera.start()
    assert waiting.wait(1), "capture worker did not start"
    stopped = threading.Event()
    thread = threading.Thread(
        target=lambda: (camera.stop(), stopped.set()), daemon=True
    )
    thread.start()
    assert stopped.wait(2), "stop() hung while frame queues were empty"
    thread.join(1)
    assert camera._pipeline is None


def test_empty_frame_queues_timeout(sdk, monkeypatch):
    monkeypatch.setattr("src.oakd.FRAME_TIMEOUT_S", 0.02)
    camera = OakD()
    assert camera.initialize_camera()
    start = time.monotonic()
    assert camera.capture_frame() is None
    assert time.monotonic() - start < 1
    camera.close_camera()


def test_sdk_point_and_quaternion_accessors():
    """Optional compatibility check against installed real SDK data types."""
    dai = pytest.importorskip("depthai")
    from datetime import timedelta

    message = SimpleNamespace(
        getTranslation=lambda: dai.Point3d(1, 2, 3),
        getQuaternion=lambda: dai.Quaterniond(0, 0, 0, 1),
        getTimestamp=lambda: timedelta(seconds=1),
    )
    camera = OakD(enable_vio=True)
    camera._vio_queue = Mock()
    camera._vio_queue.tryGet.side_effect = [message, None]
    pose = camera.capture_pose()
    assert pose.position_m == (1, 2, 3)
    assert pose.orientation_xyzw == (0, 0, 0, 1)
