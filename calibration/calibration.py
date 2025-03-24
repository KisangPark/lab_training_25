"""
camera calibration

use checkerboard & openCV to calibrate
1) find parameters
2) input real world sizes & estimate the size of the object (on same plane)
3) similar with aruco tag -> known sized tag
"""

import cv2
import numpy as np
import os
import glob

img_shape = []

# size of checkerboard
CHECKERBOARD = (6,9) #inner intersections

#criteria for terminalization
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D points save
points_3D = []
# 2D points save
points_2D = []

# make fundamental points (target?)
object_point = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
object_point[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
object_point *= 20 #20mm for one checkerboard pixel

prev_img_shape = None
# extract images
images = glob.glob('/home/kisangpark/lab_training/calibration/data/*.jpg')
print(images)
for file in images:
    print("entered")
    img = cv2.imread(file)
    # grayscale (efficiency)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    # finding corners
    ret, corners = cv2.findChessboardCorners(gray,
                                             CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    # image, patternsize, falgs
    #ret = true if number of corners are correct / corners return the list of corner coordinates                
    

    if ret == True:
        points_3D.append(object_point)

        # precision
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        # image, corners, window size, zerozone, criteria
        img_shape = gray.shape[::-1]
        print (img_shape)

        points_2D.append(corners2) # 2D points obtained by checkerboard image
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    cv2.imshow('img',img)
    cv2.waitKey(2000)
cv2.destroyAllWindows()


#Calibration!!!!!
# hand 3D points, 2D points, image size
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(points_3D, points_2D, img_shape, None, None)

print("intrinsic matrix:", mtx)
print("distortion parameters:", dist)
print("rotation vector:", rvecs)
print("transition vector:", tvecs)


#save matrix & distortion parameters with npy file
np.save("matrix.npy", mtx)
np.save("distortion.npy", dist)

a = np.load("matrix.npy")
print(a)