import detectAruco
import maths
import marker
import numpy as np

def locateMarkers(knownMarker):
    for key in marker.markers:
        mk = marker.markers[key]
        if mk.tracking and not mk.found:
            if mk.locationiters < 100:
                trandif = np.squeeze(np.asarray(np.subtract(mk.translation, knownMarker.translation)))
                rotdif = np.squeeze(np.asarray(np.subtract(mk.rotation, knownMarker.rotation)))
                xrot = marker.rotationMatrix((1,0,0), rotdif[0])
                yrot = marker.rotationMatrix((0,1,0), rotdif[1])
                zrot = marker.rotationMatrix((0,0,1), rotdif[2])
                res = trandif
                res = np.dot(xrot, res)
                res = np.dot(yrot, res)
                res = np.dot(zrot, res)
                (x,y,z) = res
                mk.findingLocations.append(res)
                mk.findingRotations.append(rotdif[1])
                mk.locationiters += 1
            if mk.locationiters == 100:
                (x,y,z) = np.average(mk.findingLocations, axis = 0)
                rot = np.average(mk.findingRotations)
                mk.world_pts = marker.makeWorldCoords((x,y,z), (0.09,0.09,0), -rot)
                mk.found = True
                print((x,y,z))
                mk.locationiters += 1
            # print(something)