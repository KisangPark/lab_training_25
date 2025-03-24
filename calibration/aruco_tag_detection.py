import numpy as np
import cv2
from cv2 import aruco

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

#marker detection -> initialize parameters for detector
detector_param = aruco.DetectorParameters_create()#DetectorParameters()

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
        for id, corner in zip(ID, corners):
            cv2.polylines(frame, [corner.astype(np.int32)], True, (0,255,255), 4, cv2.LINE_AA)
            #polyline -> given picture and points -> draw lines
            print(corner)
            corner = corner.reshape(4,2) # corners for one tag -> because, original stuff is 3 dimensional
            print(corner)
            corner = corner.astype(int)
            top_right = corner[0].ravel() #np.ravel() -> contiguous flattened array!
            top_left = corner[1].ravel()
            bottom_right = corner[2].ravel()
            bottom_left = corner[3].ravel()
            
            #put the text of top right corner on the frame image
            cv2.putText(frame, f"id: {id[0]}", #show the info of top right
            top_right, cv2.FONT_HERSHEY_PLAIN, 1.3, (200,100,0), 2, cv2.LINE_AA,)

    cv2.imshow("detection", frame)
            
    if cv2.waitKey(1) == ord("q"):
        break
            
cap.release()
cv2.destroyAllWindows()

