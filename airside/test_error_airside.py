import cv2
import yaml
import numpy as np
import os
import math
from pymavlink import mavutil
import socket
import struct
ALTITUDE=10.0
CONNECTION_STRING="/dev/ttyAMA0"

sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("100.73.30.108", 2000))


def main():
    conn=mavutil.mavlink_connection(CONNECTION_STRING)
    conn.wait_heartbeat()
    print("Connected")
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
    video_cap= cv2.VideoCapture(0)
    video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cv2.waitKey(0)
    _,frame= video_cap.read()
    dst=cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    x,y,w,h=roi
    dst_original=dst[y:y+h, x:x+w]
    gray_original=cv2.cvtColor(dst_original, cv2.COLOR_RGB2GRAY)
    if cv2.waitKey(0) & 0xFF == ord("q"):
        return
    kp1, des1 = orb.detectAndCompute(gray_original, None)
    while True:
        ret, frame= video_cap.read()
        if not ret:
            continue
        message=conn.recv_match(type="ATTITUDE",blocking=True)
        if message is None:
            continue
        roll=message.roll
        pitch=message.pitch
        message=conn.recv_match(type="GLOBAL_POSITION_INT",blocking=True)
        if message is None:
            continue
        altitude=message.relative_alt*1000
        dst=cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        x,y,w,h=roi
        dst_live=dst[y:y+h, x:x+w]
        gray_live=cv2.cvtColor(dst_live, cv2.COLOR_RGB2GRAY)
        kp2, des2 = orb.detectAndCompute(gray_live, None)
        try:
            matches = BFMatcher.match(des1, des2)
        except Exception:
            continue
        matches = sorted(matches, key=lambda x: x.distance)
        takeoff_3d_points=[]
        landing_3d_points=[]
        for match in matches:
            x_px=kp1[match.queryIdx].pt[0]-cx
            y_px=kp1[match.queryIdx].pt[1]-cy
            pz=altitude*math.cos(pitch)*math.cos(roll)
            px=(-x_px-math.sin(pitch)*fx)*pz/fx
            py=(-y_px+math.sin(roll)*fy)*pz/fy
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
        tx,ty=H[0,2],H[1,2]
        print(tx,ty)
        cv2.arrowedLine(dst_live, (int(cx), int(cy)), (int(cx+tx*250), int(cy+ty*250)), (255,0,0), 2)
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