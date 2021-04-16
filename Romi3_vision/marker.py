# holds the array that passes marker data between different areas

import numpy as np
from collections import deque
import math

class Marker:
    def __init__(self, id, world_pts):
        self.id = id
        self.tracking = False
        self.image_pts = None
        self.world_pts = world_pts
        self.rotation = None
        self.translation = None



# Return the rotation matrix associated with counterclockwise rotation about
# the given axis by theta radians.
# function from: https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
# (i cannot do this math lol)
def rotationMatrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

# utility function for generating world points of the corners of a marker
# parameters:
# point: location of top left corner (x,y,z)
# dimensions: (width, height, depth) depth will be zero
# rotation: radians to rotate so that the markers can be on different planes
def makeWorldCoords(point, dimensions, rotation):
    rotation_matrix = rotationMatrix((0,1,0), rotation)
    dimensions = np.dot(rotation_matrix, dimensions)
    # unfold for simpler access
    (x, y, z) = point
    (width, height, depth) = dimensions

    # modifications to each axis for opposite corner
    x_mod = x + width
    y_mod = y + height
    z_mod = z + depth

    #each corner's xyz coordinates
    top_left= (x, y, z)
    top_right = (x_mod, y, z_mod)
    bottom_right = (x_mod, y_mod, z_mod)
    bottom_left = (x, y_mod, z)

    #final array
    return np.float32([top_left,top_right,bottom_right,bottom_left])


# array of markers that is used through the rest of the program:
markers = {
    0: Marker(0,makeWorldCoords((0,0,0), (0.15,0.15,0), 3.14)),
    1: Marker(1,makeWorldCoords((0.36,0,0), (0.15,0.15,0), 0)),
    2: Marker(1,makeWorldCoords((0.72,0,0), (0.15,0.15,0), 0)),
}

def reset():
    for key in markers:
        markers[key].tracking = False
        markers[key].image_pts = None

def tracking(imgpts, id):
    if id in markers:
        markers[id].tracking = True
        markers[id].image_pts = imgpts