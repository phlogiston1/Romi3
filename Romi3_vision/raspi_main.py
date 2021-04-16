# main function for running on raspberry pi.
# includes cameraserver and networktables code.

import detectAruco
import golfb
import maths
import cv2
import time
from cscore import CameraServer as cs
from networktables import NetworkTables, NetworkTablesInstance

# network tables setup
ntinst = NetworkTablesInstance.getDefault()

ntinst.startClientTeam(0)
ntinst.startDSClient()

nt = NetworkTables.getTable('vision')

video_out = cs.getInstance().putVideo('Vision Magic', 320, 240)

time.sleep(0.5)
while True:
    # run detection functions:
    detectAruco.exec(False, False)
    img = golfb.detectBall(detectAruco.frame)

    #run pose finding functions:
    maths.exec(img)

    robot_location = maths.getCameraLocation()
    ball_location = golfb.getBallLocation()
    print(robot_location)

    # update networktables
    nt.putNumber('robot_x', robot_location[0])
    nt.putNumber('robot_y', robot_location[1])

    nt.putBoolean('tracking_ball', ball_location[0])
    nt.putNumber('ball_x', ball_location[1])
    nt.putNumber('ball_y', ball_location[2])

    # write cameraserver
    video_out.putFrame(img)

    cv2.waitKey(1)