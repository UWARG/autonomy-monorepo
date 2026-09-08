import cv2
import yaml
import numpy as np
import os
import math
ALTITUDE=10.0

def main():
    with open(os.path.join(os.path.dirname(__file__),"src","engine", "camera_info.yaml"), "r") as f:
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
    orb = cv2.ORB_create(nfeatures=1000)
    video_cap= cv2.VideoCapture(0)
    cv2.waitKey(0)
    _,frame= video_cap.read()
    dst=cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    x,y,w,h=roi
    dst=dst[y:y+h, x:x+w]
    gray=dst
    gray=cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
    cv2.imshow("Frame", dst)
    if cv2.waitKey(0) & 0xFF == ord("q"):
        return
    kp1, des1 = orb.detectAndCompute(gray, None)
    while True:
        ret, frame= video_cap.read()
        dst=cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        x,y,w,h=roi
        dst=dst[y:y+h, x:x+w]
        gray=cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
        if not ret:
            continue
        kp2, des2 = orb.detectAndCompute(gray, None)
        matches = BFMatcher.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        good_matches = matches[:50]
        takeoff_3d_points=[]
        landing_3d_points=[]
        for match in good_matches:
            x_px=kp1[match.queryIdx].pt[0]-cx
            y_px=kp1[match.queryIdx].pt[1]-cy
            pz=ALTITUDE
            px=(-x_px)*pz/fx
            py=(-y_px)*pz/fy
            takeoff_3d_points.append([px,py,pz])
            x_px=kp2[match.trainIdx].pt[0]-cx
            y_px=kp2[match.trainIdx].pt[1]-cy
            pz=ALTITUDE
            px=(-x_px)*pz/fx
            py=(-y_px)*pz/fy
            landing_3d_points.append([px,py,pz])
        H,inliers=cv2.estimateAffinePartial2D(
            np.asarray(takeoff_3d_points,dtype=np.float32),
            np.asarray(landing_3d_points,dtype=np.float32),
            method=cv2.RANSAC,
            ransacReprojThreshold=0.02,
            confidence=0.99
        )
        if H is None:
            continue
        tx,ty,tz=H[0,2],H[1,2],H[2,2]
        cv2.arrowedLine(gray, (int(cx), int(cy)), (int(cx+tx), int(cy+ty)), (0, 0, 255), 2)
        cv2.imshow("Frame", gray)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    video_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()