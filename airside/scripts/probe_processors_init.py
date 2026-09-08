#!/usr/bin/env python3
"""Reproduce Processor native-init sequence without ROS spin."""
import faulthandler
import sys

faulthandler.enable(file=sys.stderr, all_threads=True)

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def main() -> None:
    import numpy, scipy

    print("numpy", numpy.__version__, numpy.__file__)
    print("scipy", scipy.__version__, scipy.__file__)
    print("cv2", cv2.__version__)
    print("cuda devices", cv2.cuda.getCudaEnabledDeviceCount() if hasattr(cv2, "cuda") else None)

    print("step: ORB_create", flush=True)
    orb = cv2.ORB_create(nfeatures=1000)

    print("step: camera_intrinsics", flush=True)
    with open(os.path.join(get_package_share_directory("engine"), "camera_info.yaml"), "r") as f:
        camera_info = yaml.safe_load(f)
    k = np.array(camera_info["camera_matrix"]["data"]).reshape(3, 3)
    d = np.array(camera_info["distortion_coefficients"]["data"])
    w, h = camera_info["width"], camera_info["height"]
    new_k, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 0)
    mapx, mapy = cv2.initUndistortRectifyMap(k, d, None, new_k, (w, h), cv2.CV_32FC1)
    print("roi", roi, "maps", mapx.shape, mapy.shape, flush=True)

    print("step: CUDA BFMatcher", flush=True)
    matcher = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING)
    des = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
    a, b = cv2.cuda.GpuMat(), cv2.cuda.GpuMat()
    a.upload(des)
    b.upload(des)
    matches = matcher.match(a, b)
    print("matches", len(matches), flush=True)

    gray = np.random.randint(0, 256, (480, 640), dtype=np.uint8)
    print("step: ORB detectAndCompute", flush=True)
    kp, des2 = orb.detectAndCompute(gray, None)
    print("kp", len(kp) if kp else 0, "des", None if des2 is None else des2.shape, flush=True)
    print("OK", flush=True)


if __name__ == "__main__":
    main()
