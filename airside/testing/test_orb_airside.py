import cv2
import yaml
import numpy as np
import os
import math
from pymavlink import mavutil
import socket
import struct
import time
ALTITUDE=0.78
CONNECTION_STRING="/dev/ttyAMA0"
DISTANCE_SENSOR_HZ=10


def request_distance_sensor_stream(conn: mavutil.mavlink_connection) -> None:
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        int(1e6 / DISTANCE_SENSOR_HZ),
        0, 0, 0, 0, 0,
    )


def read_distance_m(conn: mavutil.mavlink_connection, timeout: float = 1.0) -> float | None:
    message = conn.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=timeout)
    if message is None:
        return None
    distance_m = message.current_distance / 100.0
    if not math.isfinite(distance_m) or distance_m <= 0.0:
        return None
    return distance_m

sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("100.73.30.108", 2000))

def main():
    conn=mavutil.mavlink_connection(CONNECTION_STRING)
    conn.wait_heartbeat()
    print("Connected")
    request_distance_sensor_stream(conn)
    with open(os.path.join(os.path.dirname(__file__),"camera_info.yaml"), "r") as f:
        camera_info = yaml.safe_load(f)
        height=camera_info["height"]
        width=camera_info["width"]
        distortion_model=camera_info["distortion_model"]
        k=camera_info["camera_matrix"]["data"]
        d=camera_info["distortion_coefficients"]["data"]
        r=camera_info["rectification_matrix"]["data"]
        p=camera_info["projection_matrix"]["data"]
    reshaped_k=np.asarray(k, dtype=np.float64).reshape(3, 3)
    reshaped_d=np.asarray(d, dtype=np.float64).reshape(-1)
    size=(int(width), int(height))
    new_camera_matrix,roi=cv2.getOptimalNewCameraMatrix(reshaped_k, reshaped_d, size, 0)
    mapx,mapy=cv2.initUndistortRectifyMap(
        reshaped_k, reshaped_d, None, new_camera_matrix, size, cv2.CV_32FC1
    )
    x,y,w,h=roi
    fx=new_camera_matrix[0,0]
    fy=new_camera_matrix[1,1]
    cx=new_camera_matrix[0,2]-x
    cy=new_camera_matrix[1,2]-y
    BFMatcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    if hasattr(cv2, "cuda") and cv2.cuda.getCudaEnabledDeviceCount() > 0:
        BFMatcher = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING)
    orb = cv2.ORB_create(nfeatures=10000)
    #wait for video_cap 1 or 0 to be available
    while True:
        if os.path.exists("/dev/video1"):
            video_cap= cv2.VideoCapture(1)
            break
        if os.path.exists("/dev/video0"):
            video_cap= cv2.VideoCapture(0)
            break
        time.sleep(0.1)
    video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    #warm up camera
    for _ in range(20):
        ret, frame= video_cap.read()
        if not ret:
            continue
    _,frame= video_cap.read()
    teach_altitude=read_distance_m(conn)
    attitude=conn.recv_match(type="ATTITUDE",blocking=True)
    if attitude is None:
        teach_roll=0.0
        teach_pitch=0.0
        teach_yaw=0.0
    else:
        teach_roll=attitude.roll
        teach_pitch=attitude.pitch
        teach_yaw=attitude.yaw
    print(f"Roll: {teach_roll:.2f} deg, Pitch: {teach_pitch:.2f} deg, Yaw: {teach_yaw:.2f} deg")
    if teach_altitude is None:
        teach_altitude=ALTITUDE
        print(f"No rangefinder at teach; using default {teach_altitude:.2f} m")
    else:
        print(f"Teach altitude from rangefinder: {teach_altitude:.2f} m")
    dst=cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    x,y,w,h=roi
    dst_original=dst[y:y+h, x:x+w]
    gray_original=cv2.cvtColor(dst_original, cv2.COLOR_RGB2GRAY)
    kp1, des1 = orb.detectAndCompute(gray_original, None)
    ok, res = cv2.imencode(
        ".jpg", dst_original, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    )
    if ok:
        data = res.tobytes()
        sock.sendall(struct.pack("!I", len(data)) + data)
    while True:
        time.sleep(0.5)
        ret, frame= video_cap.read()
        if not ret:
            continue
        message=conn.recv_match(type="ATTITUDE",blocking=True)
        if message is None:
            continue
        roll=message.roll
        pitch=message.pitch
        altitude=read_distance_m(conn)
        if altitude is None:
            continue
        dst=cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        x,y,w,h=roi
        dst_live=dst[y:y+h, x:x+w]
        gray_live=cv2.cvtColor(dst_live, cv2.COLOR_RGB2GRAY)
        kp2, des2 = orb.detectAndCompute(gray_live, None)
        if des1 is None or des2 is None or not kp1 or not kp2:
            continue
        try:
            matches = BFMatcher.match(des1, des2)
        except Exception:
            continue
        if len(matches) < 50:
            continue
        matches = sorted(matches, key=lambda x: x.distance)[:50]
        takeoff_3d_points=[]
        landing_3d_points=[]
        for match in matches:
            x_px=kp1[match.queryIdx].pt[0]-cx
            y_px=kp1[match.queryIdx].pt[1]-cy
            pz=teach_altitude*math.cos(teach_pitch)*math.cos(teach_roll)
            px=(-x_px-math.sin(teach_pitch)*fx)*pz/fx
            py=(-y_px+math.sin(teach_roll)*fy)*pz/fy
            takeoff_3d_points.append([px,py])
            x_px=kp2[match.trainIdx].pt[0]-cx
            y_px=kp2[match.trainIdx].pt[1]-cy
            pz=altitude*math.cos(pitch)*math.cos(roll)
            px=(-x_px-math.sin(pitch)*fx)*pz/fx
            py=(-y_px+math.sin(roll)*fy)*pz/fy
            landing_3d_points.append([px,py])
        H,inliers=cv2.estimateAffinePartial2D(
            np.asarray(takeoff_3d_points,dtype=np.float32),
            np.asarray(landing_3d_points,dtype=np.float32),
            method=cv2.RANSAC,
            ransacReprojThreshold=0.02,
            confidence=0.99
        )
        if H is None:
            continue
        tx,ty=float(H[0,2]),float(H[1,2])
        print(tx,ty)
        if not (math.isfinite(tx) and math.isfinite(ty)):
            continue
        arrow_scale=min(250.0, 100.0/max(abs(tx), abs(ty), 1e-6))
        end_x=int(round(cx+tx*arrow_scale))
        end_y=int(round(cy+ty*arrow_scale))
        end_x=max(0, min(w-1, end_x))
        end_y=max(0, min(h-1, end_y))
        cv2.arrowedLine(
            dst_live,
            (int(round(cx)), int(round(cy))),
            (end_x, end_y),
            (255, 0, 0),
            2,
        )
        ok, res = cv2.imencode(
            ".jpg", dst_live, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        )
        if not ok:
            continue
        data = res.tobytes()
        sock.sendall(struct.pack("!I", len(data)) + data)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    video_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()