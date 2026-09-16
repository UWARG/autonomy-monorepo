"""Manual test for arducam"""

import src.arducam as arducam_module

arducam_module.ARDU_DEVICE_INDEX = 1  # laptop testing only: index 0 is the built-in webcam

from src.arducam import Arducam

cam = Arducam()
print("initialize_camera():", cam.initialize_camera())

frame = cam.capture_frame()
if frame is None:
    print("capture_frame() returned None")
else:
    print("rgb shape:", frame.rgb.shape, frame.rgb.dtype)

cam.close_camera()
print("closed")
