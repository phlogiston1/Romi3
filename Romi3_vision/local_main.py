# to run the vision trackers on a local machine.
# has gui for tuning thresholding values

import detectAruco
import golfb
import maths
import copy
import cv2
import marker
import findMarkerLocations as fml

window_detection_name="golf ball threshold"
max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value


def on_lowH(value):
    golfb.lowh = value

def on_highH(value):
    golfb.highh = value

def on_lowS(value):
    golfb.lows = value

def on_highS(value):
    golfb.highs = value

def on_lowV(value):
    golfb.lowv = value

def on_highV(value):
    golfb.highv = value

cv2.namedWindow(window_detection_name)
cv2.createTrackbar("low h", window_detection_name , low_H, max_value_H, on_lowH)
cv2.createTrackbar("high h", window_detection_name , high_H, max_value_H, on_highH)
cv2.createTrackbar("low s", window_detection_name , low_S, max_value, on_lowS)
cv2.createTrackbar("high s", window_detection_name , high_S, max_value, on_highS)
cv2.createTrackbar("low v", window_detection_name , low_V, max_value, on_lowV)
cv2.createTrackbar("high v", window_detection_name , high_V, max_value, on_highV)


while True:
    detectAruco.exec(True, False)
    frame = copy.copy(detectAruco.frame)
    img = golfb.detectBall(frame)
    maths.exec(img)
    pos = maths.getCameraLocation()
    for key in marker.markers:
        mk = marker.markers[key]
        # if mk.found: fml.locateMarkers(mk)
    print(pos)

    cv2.imshow(window_detection_name, golfb.frame_threshold)
    cv2.imshow("frame", img)
    cv2.waitKey(1)