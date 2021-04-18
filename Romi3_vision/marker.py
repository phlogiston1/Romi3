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
        self.locationiters = 0
        self.findingLocations = []
        self.findingRotations = []
        self.found = False



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

def blankMarker(id):
    return Marker(id, makeWorldCoords((0,0,0), (0.09,0.09,0), 0))

# array of markers that is used through the rest of the program:
markers = {
    #front left -> right
    48: Marker(48, makeWorldCoords((0.21, -0.275, 0), (0.09, 0.09, 0), 0)),
    49: Marker(48, makeWorldCoords((0.3, -0.275, 0), (0.09, 0.09, 0), 0)),
    50: Marker(48, makeWorldCoords((0.21, -0.175, 0), (0.09, 0.09, 0), 0)),
    51: Marker(48, makeWorldCoords((0.3, -0.175, 0), (0.09, 0.09, 0), 0)),

    84: Marker(48, makeWorldCoords((0.47, -0.275, 0), (0.09, 0.09, 0), 0)),
    85: Marker(48, makeWorldCoords((0.57, -0.275, 0), (0.09, 0.09, 0), 0)),
    86: Marker(48, makeWorldCoords((0.47, -0.175, 0), (0.09, 0.09, 0), 0)),
    87: Marker(48, makeWorldCoords((0.57, -0.175, 0), (0.09, 0.09, 0), 0)),

    44: Marker(48, makeWorldCoords((0.76, -0.275, 0), (0.09, 0.09, 0), 0)),
    45: Marker(48, makeWorldCoords((0.86, -0.275, 0), (0.09, 0.09, 0), 0)),
    46: Marker(48, makeWorldCoords((0.76, -0.175, 0), (0.09, 0.09, 0), 0)),
    47: Marker(48, makeWorldCoords((0.86, -0.175, 0), (0.09, 0.09, 0), 0)),

    #left side front -> back
    88: Marker(48, makeWorldCoords((0, -0.275, -0.41), (0.09, 0.09, 0), np.radians(-90))),
    89: Marker(48, makeWorldCoords((0, -0.275, -0.31), (0.09, 0.09, 0), np.radians(-90))),
    90: Marker(48, makeWorldCoords((0, -0.175, -0.41), (0.09, 0.09, 0), np.radians(-90))),
    91: Marker(48, makeWorldCoords((0, -0.175, -0.31), (0.09, 0.09, 0), np.radians(-90))),

    52: Marker(48, makeWorldCoords((0, -0.275, -0.66), (0.09, 0.09, 0), np.radians(-90))),
    53: Marker(48, makeWorldCoords((0, -0.275, -0.56), (0.09, 0.09, 0), np.radians(-90))),
    54: Marker(48, makeWorldCoords((0, -0.175, -0.66), (0.09, 0.09, 0), np.radians(-90))),
    55: Marker(48, makeWorldCoords((0, -0.175, -0.56), (0.09, 0.09, 0), np.radians(-90))),

    32: Marker(48, makeWorldCoords((0, -0.275, -0.935), (0.09, 0.09, 0), np.radians(-90))),
    33: Marker(48, makeWorldCoords((0, -0.275, -0.835), (0.09, 0.09, 0), np.radians(-90))),
    34: Marker(48, makeWorldCoords((0, -0.175, -0.935), (0.09, 0.09, 0), np.radians(-90))),
    35: Marker(48, makeWorldCoords((0, -0.175, -0.835), (0.09, 0.09, 0), np.radians(-90))),

    96: Marker(48, makeWorldCoords((0, -0.275, -1.035), (0.09, 0.09, 0), np.radians(-90))),
    97: Marker(48, makeWorldCoords((0, -0.275, -0.935), (0.09, 0.09, 0), np.radians(-90))),
    98: Marker(48, makeWorldCoords((0, -0.175, -1.035), (0.09, 0.09, 0), np.radians(-90))),
    99: Marker(48, makeWorldCoords((0, -0.175, -0.935), (0.09, 0.09, 0), np.radians(-90))),

    64: Marker(48, makeWorldCoords((0, -0.275, -1.255), (0.09, 0.09, 0), np.radians(-90))), #1.59
    65: Marker(48, makeWorldCoords((0, -0.275, -1.155), (0.09, 0.09, 0), np.radians(-90))),
    66: Marker(48, makeWorldCoords((0, -0.175, -1.255), (0.09, 0.09, 0), np.radians(-90))),
    67: Marker(48, makeWorldCoords((0, -0.175, -1.155), (0.09, 0.09, 0), np.radians(-90))),

    56: Marker(48, makeWorldCoords((0, -0.275, -1.59), (0.09, 0.09, 0), np.radians(-90))), #1.59
    57: Marker(48, makeWorldCoords((0, -0.275, -1.49), (0.09, 0.09, 0), np.radians(-90))),
    58: Marker(48, makeWorldCoords((0, -0.175, -1.59), (0.09, 0.09, 0), np.radians(-90))),
    59: Marker(48, makeWorldCoords((0, -0.175, -1.49), (0.09, 0.09, 0), np.radians(-90))),

    #back left->right | 2.286 meteres back for corner
    60: Marker(48, makeWorldCoords((0.963, -0.275, -2.286), (0.09, 0.09, 0), np.radians(180))), #0.913
    61: Marker(48, makeWorldCoords((0.953, -0.275, -2.286), (0.09, 0.09, 0), np.radians(180))),
    62: Marker(48, makeWorldCoords((0.963, -0.175, -2.286), (0.09, 0.09, 0), np.radians(180))),
    63: Marker(48, makeWorldCoords((0.953, -0.175, -2.286), (0.09, 0.09, 0), np.radians(180))),

    68: Marker(48, makeWorldCoords((0.693, -0.275, -2.286), (0.09, 0.09, 0), np.radians(180))), #0.913
    69: Marker(48, makeWorldCoords((0.683, -0.275, -2.286), (0.09, 0.09, 0), np.radians(180))),
    70: Marker(48, makeWorldCoords((0.693, -0.175, -2.286), (0.09, 0.09, 0), np.radians(180))),
    71: Marker(48, makeWorldCoords((0.683, -0.175, -2.286), (0.09, 0.09, 0), np.radians(180))),

    92: Marker(48, makeWorldCoords((0.42, -0.275, -2.286), (0.09, 0.09, 0), np.radians(180))), #0.913
    93: Marker(48, makeWorldCoords((0.32, -0.275, -2.286), (0.09, 0.09, 0), np.radians(180))),
    94: Marker(48, makeWorldCoords((0.42, -0.175, -2.286), (0.09, 0.09, 0), np.radians(180))),
    95: Marker(48, makeWorldCoords((0.32, -0.175, -2.286), (0.09, 0.09, 0), np.radians(180))),

    #right side front -> back

    80: Marker(48, makeWorldCoords((1.15, -0.275, -0.24), (0.09, 0.09, 0), np.radians(90))), #0.913
    81: Marker(48, makeWorldCoords((1.15, -0.275, -0.34), (0.09, 0.09, 0), np.radians(90))),
    82: Marker(48, makeWorldCoords((1.15, -0.175, -0.24), (0.09, 0.09, 0), np.radians(90))),
    83: Marker(48, makeWorldCoords((1.15, -0.175, -0.34), (0.09, 0.09, 0), np.radians(90))),

    40: Marker(48, makeWorldCoords((1.15, -0.275, -0.59), (0.09, 0.09, 0), np.radians(90))), #0.913
    41: Marker(48, makeWorldCoords((1.15, -0.275, -0.69), (0.09, 0.09, 0), np.radians(90))),
    42: Marker(48, makeWorldCoords((1.15, -0.175, -0.59), (0.09, 0.09, 0), np.radians(90))),
    43: Marker(48, makeWorldCoords((1.15, -0.175, -0.69), (0.09, 0.09, 0), np.radians(90))),

    76: Marker(48, makeWorldCoords((1.15, -0.275, -0.97), (0.09, 0.09, 0), np.radians(90))), #0.913
    77: Marker(48, makeWorldCoords((1.15, -0.275, -1.07), (0.09, 0.09, 0), np.radians(90))),
    78: Marker(48, makeWorldCoords((1.15, -0.175, -0.97), (0.09, 0.09, 0), np.radians(90))),
    79: Marker(48, makeWorldCoords((1.15, -0.175, -1.07), (0.09, 0.09, 0), np.radians(90))),

    36: Marker(48, makeWorldCoords((1.15, -0.275, -1.32), (0.09, 0.09, 0), np.radians(90))), #0.913
    37: Marker(48, makeWorldCoords((1.15, -0.275, -1.42), (0.09, 0.09, 0), np.radians(90))),
    38: Marker(48, makeWorldCoords((1.15, -0.175, -1.32), (0.09, 0.09, 0), np.radians(90))),
    39: Marker(48, makeWorldCoords((1.15, -0.175, -1.42), (0.09, 0.09, 0), np.radians(90))),

    72: Marker(48, makeWorldCoords((1.15, -0.275, -1.75), (0.09, 0.09, 0), np.radians(90))), #0.913
    73: Marker(48, makeWorldCoords((1.15, -0.275, -1.85), (0.09, 0.09, 0), np.radians(90))),
    74: Marker(48, makeWorldCoords((1.15, -0.175, -1.75), (0.09, 0.09, 0), np.radians(90))),
    75: Marker(48, makeWorldCoords((1.15, -0.175, -1.85), (0.09, 0.09, 0), np.radians(90))),

    0: Marker(48, makeWorldCoords((1.15, -0.275, -2.03), (0.09, 0.09, 0), np.radians(90))), #0.913
    1: Marker(48, makeWorldCoords((1.15, -0.275, -2.13), (0.09, 0.09, 0), np.radians(90))),
    2: Marker(48, makeWorldCoords((1.15, -0.175, -2.03), (0.09, 0.09, 0), np.radians(90))),
    3: Marker(48, makeWorldCoords((1.15, -0.175, -2.13), (0.09, 0.09, 0), np.radians(90))),
}

def reset():
    for key in markers:
        markers[key].tracking = False
        markers[key].image_pts = None

def tracking(imgpts, id):
    if id in markers:
        markers[id].tracking = True
        markers[id].image_pts = imgpts