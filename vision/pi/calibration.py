import numpy as np
import cv2 as cv
import glob

#distortionMatrix =  [[682.98262078   0.         308.67989006][  0.         679.09669922 226.69400685][  0.           0.           1.        ]]


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
cap = cv.VideoCapture(1)
images = []
while images.__len__() < 30:
    success, newImg = cap.read()
    if not success:
        continue
    # Find the chess board corners
    gray = cv.cvtColor(newImg, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        cv.drawChessboardCorners(newImg, (7,6), corners2, ret)
        images.append(newImg)

    # Draw and display the corners
    cv.imshow('img', newImg)
    cv.waitKey(60)

cv.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
img = images[1]
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
print(newcameramtx)