from oakd import Oakd

if __name__ == "__main__":
    cam = Oakd(640, 400, 20, slam_enabled=True, rerun_enabled=True)
    cam.start()
