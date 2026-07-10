from oakd import Oakd
import cv2
import os
import platform

if platform.system() == "Linux":
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    os.environ['QT_QPA_FONTDIR'] = '/usr/share/fonts/open-sans/'

if __name__ == "__main__":
    cam = Oakd(640, 400, 20, slam_enabled=True, rerun_enabled=True)
    cam.start()

    # cam = Oakd(640, 400, 20)
    # cam.start()
    # frame = cam.capture_frame().rgb
    # cv2.imshow("Heh", frame)
    # cv2.waitKey(0)



