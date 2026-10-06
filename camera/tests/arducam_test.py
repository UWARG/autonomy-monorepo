"""Manual test for arducam"""

import src.arducam as arducam_module

arducam_module.ARDU_DEVICE_INDEX = 1

cam = arducam_module.Arducam()
print("initialize_camera():", cam.initialize_camera())

frame = cam.capture_frame()
if frame is None:
    print("capture_frame() returned None")
else:
    print("rgb shape:", frame.rgb.shape, frame.rgb.dtype)

cam.close_camera()
print("closed")
