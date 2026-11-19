"""Capture a color point cloud from an OAK-D and export it as PLY."""

import argparse
from pathlib import Path
from time import monotonic, sleep

import numpy as np


def capture_point_cloud(
    timeout: float = 10.0, warmup: float = 2.0
) -> tuple[np.ndarray, np.ndarray]:
    """Return valid XYZ coordinates in meters and matching uint8 RGB colors.

    Frames arriving within ``warmup`` seconds of start are discarded: the first
    frames come before stereo and auto-exposure settle and contain only a few
    hundred bogus points.
    """
    import depthai as dai

    size = (640, 400)
    with dai.Pipeline() as pipeline:
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        left.requestOutput(size).link(stereo.left)
        right.requestOutput(size).link(stereo.right)

        rgb = color.requestOutput(
            size, dai.ImgFrame.Type.RGB888i, enableUndistortion=True
        )
        cloud = pipeline.create(dai.node.PointCloud)
        cloud.initialConfig.setLengthUnit(dai.LengthUnit.METER)

        if pipeline.getDefaultDevice().getPlatform() == dai.Platform.RVC4:
            align = pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(align.input)
            rgb.link(align.inputAlignTo)
            align.outputAligned.link(cloud.inputDepth)
        else:
            rgb.link(stereo.inputAlignTo)
            stereo.depth.link(cloud.inputDepth)
        rgb.link(cloud.inputColor)

        queue = cloud.outputPointCloud.createOutputQueue(maxSize=1, blocking=False)
        pipeline.start()
        started = monotonic()
        settled = started + warmup
        deadline = settled + timeout
        while pipeline.isRunning() and monotonic() < deadline:
            message = queue.tryGet()
            if message is None or monotonic() < settled:
                sleep(0.01)
                continue

            xyz, rgba = message.getPointsRGB()
            valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
            if valid.any():
                return xyz[valid], rgba[valid, :3]

    raise RuntimeError("No valid OAK-D point cloud received before capture ended.")


def write_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    """Write an XYZ/RGB vertex list in binary little-endian PLY format."""
    vertices = np.empty(
        len(xyz),
        dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("red", "u1"), ("green", "u1"), ("blue", "u1"),
        ],
    )
    vertices["x"], vertices["y"], vertices["z"] = xyz.T
    vertices["red"], vertices["green"], vertices["blue"] = rgb.T
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        "comment XYZ coordinates in meters\n"
        f"element vertex {len(vertices)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    with path.open("wb") as output:
        output.write(header.encode("ascii"))
        output.write(vertices.tobytes())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output PLY file")
    parser.add_argument(
        "--warmup", type=float, default=2.0,
        help="Seconds of frames to discard while the camera settles (default: 2)",
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="Seconds to wait for a valid frame after warm-up (default: 10)",
    )
    args = parser.parse_args()

    try:
        xyz, rgb = capture_point_cloud(timeout=args.timeout, warmup=args.warmup)
        write_ply(args.output, xyz, rgb)
    except (ImportError, RuntimeError, OSError) as error:
        parser.exit(1, f"Point-cloud capture failed: {error}\n")

    print(f"Saved {len(xyz):,} points to {args.output}")


if __name__ == "__main__":
    main()
