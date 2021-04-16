# holds functions to get camera location from marker corners

import detectAruco as dt
import undistort as ud
import marker
import numpy as np
import cv2

# get camera data from undistort
criteria = ud.criteria
camera_matrix = ud.mtx
dist = ud.dist

#points for drawn axes when doing pretty image
axis = np.float32([[0,0,0], [0.15,0,0], [0,0.15,0], [0,0,0.15]]).reshape(-1,3)
hasNewLocation = False

# get pose of a Marker object in world coordinates
def getWorldPose(tracked_marker):
    objp = tracked_marker.world_pts
    imgp = tracked_marker.image_pts

    # refine corner locations:
    corners = cv2.cornerSubPix(dt.gray,imgp,(5,5),(-1,-1),criteria)

    # run the PnP solver (whatever that is)
    (success, rotation, translation) = cv2.solvePnP(objp, corners, camera_matrix, dist)

    return (success, rotation, translation)

# draw nice axes markers on marker for visualization
def draw(img, corners, rotation, translation):
    imgpts, jac = cv2.projectPoints(axis, rotation, translation, camera_matrix, dist)
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[2].ravel()), (0,255,0), 5)
    img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel()), (255,0,0), 5)
    img = cv2.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[3].ravel()), (0,0,255), 5)
    return img

# get rotation and translation for a marker
def execForMarker(markerID):
    #safety checks...
    if not markerID in marker.markers: return None
    tracked_marker = marker.markers[markerID]
    if not tracked_marker.tracking: return None

    (_, rotation, translation) = getWorldPose(tracked_marker)
    return (rotation, translation)

# find rotation and translation matrices for all markers
def exec(img = None):
    for key in marker.markers:
        data = execForMarker(key)
        if data is not None:
            (rotation, translation) = data

            # write data to marker objects
            marker.markers[key].rotation = rotation
            marker.markers[key].translation = translation

            #draw markers on image
            if img is not None: img = draw(img, marker.markers[key].image_pts, rotation, translation)

# get camera location from a single set of rotation and translation matrices
# once again, this math is way over my head so...
# function credit: https://stackoverflow.com/questions/44726404/camera-pose-from-solvepnp
def getCameraLocationForMarker(tracked_marker):
    rotation_matrix = cv2.Rodrigues(tracked_marker.rotation)[0]
    camera_pos = -np.matrix(rotation_matrix) * np.matrix(tracked_marker.translation)
    return camera_pos

# get camera location for each marker and then average the results:
def getCameraLocation():
    pos = [0,0,0]
    num = 0
    for key in marker.markers:
        tracked_marker = marker.markers[key]
        if(tracked_marker.tracking):
            marker_location = getCameraLocationForMarker(tracked_marker)
            pos[0] += marker_location[0]
            pos[1] += marker_location[1]
            pos[2] += marker_location[2]
            num+=1
    if num > 0:
        pos[0] /= num
        pos[1] /= num
        pos[2] /= num

    return pos

def test():
    while True:
        dt.exec(False, False)
        img = dt.frame
        exec(img=img)
        pos = getCameraLocation()
        # print(pos)
        print((pos[0], pos[1], pos[2]))
        cv2.imshow("frame", img)
        cv2.waitKey(1)

if __name__ == "main":
    test()