# detects golf ball using blob detection,
# then uses golfb_math to find x, y location of ball in world coordinates relative to camera location

from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
import golfb_math as gb_m

ballsize = 0.05 # meters

#thresholding values (unpacking for better readability):
(lowh, highh) = (0, 27)
(lows, highs) = (169, 255)
(lowv, highv) = (52, 255)

#set up blob detector paramaters:
params = cv.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 1500
params.maxArea = 50000 #basically as big as possible - to override default limit

# Set up the detector
detector = cv.SimpleBlobDetector_create(params)

tracking = False
frame_threshold = None
blob = None
locationToCamera = None

def threshold(frame):
    global frame_threshold
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    frame_threshold = ~ cv.inRange(frame_HSV, (lowh, lows, lowv), (highh, highs, highv))

    # dilate threshold
    kernel = np.ones((5,5), np.uint8)
    thresh_dilation = cv.erode(frame_threshold, kernel, iterations=1) # eroding because image inverted so looking for dark areas
    frame_threshold = thresh_dilation # so can see dilation results in 'main' program threshold view window
    return frame_threshold

# run blob detection on frame
def detectBall(frame, drawframe = None):
    global frame_threshold
    global blob
    global tracking
    global locationToCamera

    thresh_dilation = threshold(frame)

    # Detect blobs.
    keypoints = detector.detect(thresh_dilation)

    if len(keypoints) > 0:
        blob = keypoints[0]
        tracking = True
    else:
        tracking = False

    # find location of ball relative to camera
    if blob is not None:
        # print(gb_m.calcFocalLength(ballsize, blob.size, 30))
        locationToCamera = gb_m.ballPointRelCamera(blob)

    #draw circles around blobs
    if drawframe is None: drawframe = frame
    im_with_keypoints = cv.drawKeypoints(drawframe, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return im_with_keypoints

# return the location of the tracked ball in a simple format
def getBallLocation():
    if locationToCamera is None: return (False, 0, 0)
    (x,y) = locationToCamera
    return (tracking, x, y)
