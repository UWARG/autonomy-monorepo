import path
import os
import cv2
import numpy as np
import glob

lowe_ratio=0.55

#bfmatcher
bf=cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
orb=cv2.ORB_create()
files=glob.glob("assets/*_1.png")
files2=glob.glob("assets/*_2.png")
files.sort()
files2.sort()
count=0
for file1,file2 in zip(files,files2):
    image1=cv2.imread(file1, cv2.IMREAD_GRAYSCALE)
    image2=cv2.imread(file2, cv2.IMREAD_GRAYSCALE)
    kp1,des1=orb.detectAndCompute(image1, None)
    kp2,des2=orb.detectAndCompute(image2, None)
    matches=bf.knnMatch(des1,des2,k=2)
    matches=[match[0] for match in matches if len(match)==2 and match[0].distance<lowe_ratio*match[1].distance]
    matches=sorted(matches, key=lambda match: match.distance)
    matches=matches[:100]
    print(f"Image: {file1}, Features: {len(kp1)}, {file2}, Features: {len(kp2)}")
    print(f"Matches: {len(matches)}")
    img_out=cv2.drawMatches(image1,kp1,image2,kp2,matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(f"results/matches_{count}{os.path.basename(file1)}", img_out)
    count+=1
