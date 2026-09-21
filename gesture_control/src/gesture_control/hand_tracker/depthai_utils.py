import depthai as dai


def probe_xyz_device(device_factory=dai.Device):
    """Return stereo availability and RGB calibration focus from one device session."""
    with device_factory() as device:
        cameras = device.getConnectedCameras()
        has_stereo = (
            dai.CameraBoardSocket.LEFT in cameras
            and dai.CameraBoardSocket.RIGHT in cameras
        ) or (
            dai.CameraBoardSocket.CAM_B in cameras
            and dai.CameraBoardSocket.CAM_C in cameras
        )
        if not has_stereo:
            return False, None

        calibration = device.readCalibration()
        return True, calibration.getLensPosition(dai.CameraBoardSocket.RGB)
