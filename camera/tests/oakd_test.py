"""OAK-D Hardware Check"""
import numpy as np

from src.oakd import OakD

cam = OakD()
print("initialize_camera():", cam.initialize_camera())

frame = cam.capture_frame()
if frame is None:
    print("capture_frame() returned None")
else:
    print("rgb shape:", frame.rgb.shape, frame.rgb.dtype)
    depth = frame.depth
    print("depth shape:", None if depth is None else depth.shape)
    print("centre_depth:", frame.centre_depth)
    if depth is not None:
        valid = depth[depth > 0]
        print("depth valid pixels:", valid.size, "/", depth.size)
        if valid.size:
            print("depth min/max/median (mm):", valid.min(), valid.max(), np.median(valid))

cam.close_camera()
print("closed")
