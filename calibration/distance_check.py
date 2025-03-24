"""
1) find aruco tag
2) using pre-defined length, intrinsic matrix, and distortion -> achieve vectors
3) visualize

aruco.estimatePoseSingleMarkers(corners, length, inner_mat, distortion) -> returns rotation, transition, objectpoints
"""
import numpy as np
import cv2
from cv2 import aruco

#load calibration data -> saved from calibration.py
inner_matrix = np.load("matrix.npy")
distortion = np.load("distortion.npy")

real_size = 7 #in milimeters

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

#marker detection -> initialize parameters for detector
detector_param = aruco.DetectorParameters_create()

#now -> capture video, use aruco detector to get vertices
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    #optional -> make grayscale to achieve afficiency
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #return corners and IDs using aruco detector
    corners, ID, reject = aruco.detectMarkers(frame_gray, arucodict, parameters=detector_param)
    #parameter defined above is used!
    #corner returned as 4xn (4 corners for one tag)

    #if markers exist, get polylines, visualize
    if corners:
        #index for vectors
        i=0

        #get rotation & transition value from aruco tag
        rotation, transition, _ = aruco.estimatePoseSingleMarkers(corners, real_size, inner_matrix, distortion)

        for id, corner in zip(ID, corners): #zip: fusion of iterable objects
            cv2.polylines(frame, [corner.astype(np.int32)], True, (0,255,255), 4, cv2.LINE_AA)
            #polyline -> given picture and points -> draw lines

            corner = corner.reshape(4,2) # corners for one tag -> because, original stuff is 3 dimensional
            corner = corner.astype(int)
            top_right = corner[0].ravel() #np.ravel() -> contiguous flattened array!
            top_left = corner[1].ravel()
            bottom_right = corner[2].ravel()
            bottom_left = corner[3].ravel()

            #distance from the camera
            distance = np.sqrt(transition[i][0][0]**2 + transition[i][0][1]**2 + transition[i][0][2]**2)
            
            #put the text of top right corner on the frame image
            #cv2.putText(frame, f"id: {id[0]}", #show the info of top right
            #top_right, cv2.FONT_HERSHEY_PLAIN, 1.3, (200,100,0), 2, cv2.LINE_AA,)
            cv2.putText(frame, f"distance: {distance}", top_right, cv2.FONT_HERSHEY_PLAIN, 2.0, (200,100,0), 2, cv2.LINE_AA,)
            #cv2.putText(frame, f"transition -> x: {transition[i][0][0]}, y:{transition[i][0][1]}", top_left, cv2.FONT_HERSHEY_PLAIN, 1.3, (200,100,0), 2, cv2.LINE_AA,)
            #cv2.putText(frame, f"rotation vector -> x: {rotation[i][0][0]}, y:{rotation[i][0][1]}", bottom_right, cv2.FONT_HERSHEY_PLAIN, 1.3, (200,100,0), 2, cv2.LINE_AA,)
            
            i+=1

    cv2.imshow("detection", frame)
            
    if cv2.waitKey(1) == ord("q"):
        break
            
cap.release()
cv2.destroyAllWindows()

