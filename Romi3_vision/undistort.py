import cv2
import numpy as np
import glob
import os
dir_path = os.path.dirname(os.path.realpath(__file__))


print("starting undistort")
print(" initializing")
chessboard_scale_unit = 1

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((10*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:10].T.reshape(-1,2)
# print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images_dir = dir_path + '/calib_photos/*'#'C:/Users/PEACCE/Documents/sean/python/opencv/romi_aruco_targets/calib_photos/*'
print(" finding photos in " + images_dir)
images = glob.glob(images_dir)
num = 1
print(" finding patterns:")
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,10), None)
    # print(corners)
    # If found, add object points, image points (after refining them)
    if ret == True:
        # print("yay")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7,6), corners2, ret)
        #cv2.imshow('calib_pattern', img)
        cv2.waitKey(1)
    print(str(num) + "/" + str(len(images)))
    num += 1

print(" running calibration")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

img = cv2.imread(images[0])
newcameramtx = None
print("ready")
def undistort(img):
    h,  w = img.shape[:2]
    global newcameramtx
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

cv2.imwrite('calibresult.png', undistort(img))

cv2.destroyAllWindows()