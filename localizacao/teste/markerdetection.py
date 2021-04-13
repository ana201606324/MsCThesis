import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

#---180 degree rotation matrix around the x axis
#R_flip = np.zeros((3,3), dtype=np.float32)
#R_flip[0,0] = 1.0
#R_flip[1,1] = 1.0
#R_flip[2,2] = 1.0

#--- define the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

#--- draw marker
img = aruco.drawMarker(aruco_dict, 43, 200);
cv2.imshow("43.png", img);
cv2.imwrite("43.png", img)
cv2.waitKey()

parameters = aruco.DetectorParameters_create()

#--- import image and create opencv image

img_file = "image2.jpg"
img = cv2.imread(img_file)



#--- convert in grayscale

gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#--- find the markers

corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

#--- draw the markers

result = aruco.drawDetectedMarkers(img, corners, ids)
cv2.imshow('Display', result)
cv2.waitKey()