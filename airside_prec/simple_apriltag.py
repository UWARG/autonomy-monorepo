import cv2
from pupil_apriltags import Detector
import yaml
import numpy as np
import time
import os
detector=Detector(families="tag36h11",nthreads=1,quad_decimate=1.0,quad_sigma=0.0,refine_edges=1,decode_sharpening=0.25,debug=0)

with open(os.path.join(os.path.dirname(__file__),"src","camera_info.yaml"),"r") as f:
    camera_params=yaml.safe_load(f)
    camera_matrix=np.array(camera_params["camera_matrix"]["data"]).reshape(3,3)
    dist_coeffs=np.array(camera_params["distortion_coefficients"]["data"]).reshape(1,14)

h=0.165
w=0.165
object_points=np.array([[-h/2,w/2,0],[h/2,w/2,0],[h/2,-w/2,0],[-h/2,-w/2,0]],dtype=np.float32)

cap=cv2.VideoCapture(0)
if not cap.isOpened():
    exit()
while True:
    ret,frame=cap.read()
    if not ret:
        break
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    results=detector.detect(gray)
    if results:
        for result in results:
            success,rvec,tvec=cv2.solvePnP(object_points,result.corners,camera_matrix,dist_coeffs)
            if success:
                print(tvec, f"tag id: {result.tag_id}")
            #cv2.rectangle(frame,tuple(result.corners[0].astype(int)),tuple(result.corners[2].astype(int)),(0,255,0),2)
    #cv2.imshow("frame",frame)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break
    time.sleep(0.1)
cap.release()
cv2.destroyAllWindows()