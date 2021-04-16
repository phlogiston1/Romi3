# holds functions for calculating location of golf ball

import numpy as np
import math

focalLength = 555
ballSize = 0.05 # meters
fov = 70
frame_w = 620
frame_h = 480


def calcFocalLength(realSize, apparentSize, distFromCamera):
    return (apparentSize * distFromCamera) / realSize

def distToCamera(realSize, apparentSize):
    return (realSize * focalLength)/apparentSize

def distToBlob(blob):
    if blob is None: return None
    return distToCamera(ballSize, blob.size)

def angleToPoint(pt):
    (x,y) = pt
    x -= frame_w/2
    ang_per_pixel = fov/frame_w #is dis right way round??
    return x * ang_per_pixel

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def ballPointRelCamera(blob):
    dist = distToBlob(blob)
    ang = angleToPoint(blob.pt)
    ang = math.radians(ang)
    (x,y) = pol2cart(dist,ang)
    return (x, y)

