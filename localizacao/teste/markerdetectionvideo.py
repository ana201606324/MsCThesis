import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import socket


#---180 degree rotation matrix around the x axis
#R_flip = np.zeros((3,3), dtype=np.float32)
#R_flip[0,0] = 1.0
#R_flip[1,1] = 1.0
#R_flip[2,2] = 1.0

#cap=cv2.VideoCapture(0) #with camera

cap=cv2.VideoCapture('video.avi')

#--- define the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

#--- draw marker
#img = aruco.drawMarker(aruco_dict, 23, 200);
#cv2.imshow("markercreated.png", img);
#cv2.waitKey()

parameters = aruco.DetectorParameters_create()

while(cap.isOpened()):
    ret, frame = cap.read()
    gray= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    
  #  if ids != None
        #ret = aruco.estimatePoseSingleMarkers(corners, marker_size 
    

    #--- draw the markers

    aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow('frame', frame)
    
    #create UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #bind the socket to the port
    server_address = (ip, port)
    s.bind(server_address)
    s.sendto(send_data.encode('utf-8'), address)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
