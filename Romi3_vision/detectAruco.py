# find all aruco markers and put they're xy coordinates into the list of markers in marker.py
# this file started as a copy of the code from this blog post: https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
# but has been so modified that it would be unrecognizable

import cv2
import time
import sys

import numpy as np
import undistort
import marker

def showFrame(img):
    cv2.imshow("Frame", img)
    key = cv2.waitKey(1) & 0xFF

print("starting detectAruco")
print(" getting aruco dictionary")
# change for different markers:
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create()

print(" initializing video stream")
vs = cv2.VideoCapture(1)
print(" waiting for things to start up")
time.sleep(1.0)

thresh_min = 200
print(" using thresholding value of: ")
print(thresh_min)

print("finished")
print("detectAruco ready")

# markers = {} #should be unused, left just in case
# points = {} #same as above
gray = None
corner = None
frame = None
corners = None

def getFrame(thresh):
    global frame
    global gray

    _, frame = vs.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180) #!!!! UNTESTEDDDDDD TODO rotation
    frame = undistort.undistort(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # if thresh parameter is true, use thresholded image
    if(thresh):
        #threshold to clean up blooming from light from retroreflector
        _, frame = cv2.threshold(gray, thresh_min, 255, cv2.THRESH_BINARY)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) #aruco detection only works with color

    return (frame, gray)

def draw(markerCorner, markerID):
    corners = markerCorner.reshape((4, 2)) # shape so that lines up with format (topLeft, topRight, bottomRight, bottomLeft)

    corners = np.array(corners).astype(int)
    (topLeft,_,_,_) = corners #need top left corner still for writing text

    corners = corners.reshape((-1,1,2))

    col = (0,0,255)
    if markerID in marker.markers: col = (0,255,0)
    cv2.polylines(frame, [corners], True, col, 2)

    # draw the ArUco marker ID on the frame
    cv2.putText(frame, str(markerID),
        (topLeft[0], topLeft[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)

    return frame


# search for markers, and put their coordinates into marker.py
def exec(show, thresh):
    # global markers
    global gray
    global frame
    global corners

    # sets all markers to 'not tracking'
    marker.reset()

    getFrame(thresh)

    #find aruco markers:
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    if(len(corners) < 1): return # end function if no matches

    ids = ids.flatten()

    for(markerCorner, markerID) in zip(corners, ids):
        corners = markerCorner.reshape((4, 2))

        #add marker to marker list
        marker.tracking(corners, markerID)
        if(show): frame = draw(markerCorner, markerID) # if this program is in charge of showing the image

    if(show): showFrame(frame)
    return frame
