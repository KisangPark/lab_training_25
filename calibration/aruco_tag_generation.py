import numpy as np
import cv2

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

#make tag from dictionary, id 1 and size 1000
size=1000
id=4
tag = np.zeros((size,size,1), dtype="uint8") #make image matrix (binary)
cv2.aruco.drawMarker(arucodict, id, size, tag, 1) #what is 1?

#save image
cv2.imwrite("aruco_tag_example4.jpg", tag)
cv2.imshow("test", tag)

cv2.waitKey(0)
cv2.destroyAllWindows()