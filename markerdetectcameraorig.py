"""
This demo calculates multiple things for different scenarios.
Here are the defined reference frames:
TAG:
                A y
                |
                |q
                |tag center
                O---------> x
CAMERA:
                X--------> x
                | frame center
                |
                |
                V y
F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis
The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)
We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import socket


#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

marker_size=11.3 #cm
xo=0
yo=0
zo=0#translaçao quando colocar mapa
#UDP socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ip='192.168.1.95'
port = 9800
address = (ip, port)


# Camera Calibration
calib_path=""
camera_matrix = np.loadtxt(calib_path+'camera_matrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'camera_dist.txt', delimiter=',')

#---180 degree rotation matrix around the x axis because the camera and the tag's referential does not match
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- matriz de transformacao homogenea composta por uma matriz de rotaçao igual a R_flip
#(pois o referencial 0 tem o eixo de y rodado 180 grau em relacao ao y da camara) e uma matriz de translaçao 
Hc0 = np.zeros((4,4), dtype=np.float32)
Hc0[0,0] = 1.0
Hc0[1,1] =-1.0
Hc0[2,2] =-1.0
Hc0[3,3] =1.0
Hc0[0,3] =xo
Hc0[1,3] =yo
Hc0[2,3] =zo

#--- define the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

#--- draw marker
#img = aruco.drawMarker(aruco_dict, 23, 200);
#cv2.imshow("markercreated.png", img);
#cv2.waitKey()

parameters = aruco.DetectorParameters_create()

#--- import image and create opencv image

#img_file = "image.png"
#img = cv2.imread(img_file)

#---capture the videocamera
cap = cv2.VideoCapture(0)

#---set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:

    #-- read camera frame
    
    ret, frame = cap.read()
    
    #--- convert in grayscale

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #--- find the markers

    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    #print(ids)
    
    if ids is not None:         
        
        #--- draw the markers and reference frame over image
        aruco.drawDetectedMarkers(frame, corners)
        #rvec rotation vectors tvec translation vectors
        rvec, tvec=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        #rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
        #print('rvec ')
        #print(rvec)
        #print('tvec ')
        #print(tvec)
        #print(len(ids))
        message = ''
        
        
        for i in range(len(ids)):
            #print(tvec[i])
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec[i-1], tvec[i-1], 10)
                       
            #-- position in camera frame
            aux = tvec[i][0].astype(float)
            pc = np.ones((4,1), dtype=np.float32)
            pc[0,0]=aux[0]
            pc[1,0]=aux[1]
            pc[2,0]=aux[2]
            
            #print(Hc0)
            #print(pc)
            
            p0=np.matmul(Hc0,pc)
            #print(p0)
            
            str_pos = "i%dx%.2fy%.2fz%.2f" % (ids[i], p0[0], p0[1], p0[2])
            
            
            #obtain rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec[i][0])[0]) 
            R_tc = R_ct.T                        
            
            #get the rotation in terms of euler 321 (needs to be flipped first: 180 degrees rotation)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
            
            #print marker's attitude respect to the camera frame
            str_ang = "t%.2f;"%(math.degrees(yaw_marker))
            #print(str_ang)
            
            message=message+(str_pos + str_ang) 
            print(message)
            
        #cv2.putText(frame, str_attitude, (0,150), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2, cv2.LINE_AA)
        
        #send UDP packet
        #print(message)
        messageb = str.encode(message)
        sock.sendto(messageb,(ip, port))
    
    #--display the frame
    
    cv2.imshow('frame', frame)
        
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        sock.close()            
        break

