import cv2
import math
import camera
import numpy as np
import time
from time import sleep
from timeit import default_timer as timer
import sys
import os



piCamera2Found = False
cam = camera.Camera(0, robottype='arlo', useCaptureThread=False) # Initalise camera
sleep(EPS)


sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# import drive and robot from lib
from lib import drive
from lib import robot



# Constants
EPS         = 0.05     # EPS for sleep time so arlo hsa time to catch up
SIGMA_THETA = 0.029320 # Sigma noise for when turning
SIGMA       = 2.0      # Sigma for driving distance


# Landmarks and maps
MAPSIZE  = [[-120, 620], [-120, 520]] # Size of the map
landmarkIDs = [1,2,3,4]
## Landmark Coordinates
landmarks = {
    1: (0.0, 0.0),
    2: (0.0, 300.0),
    3: (400.0, 0.0),
    4: (400.0, 3.0),
}

## What landmarks have been visited?
VisitedLandmark = {
    1: False,
    2: False,
    3: False,
    4: False
}


# We know that obstacle landmarks are > 4









def drivetol1():
    colour = cam.get_next_frame() # Capture image
    sleep(EPS)
    objectIDs, dists, angles = cam.detect_aruco_objects(colour) # Return ID's
    sleep(EPS)

    # A landmark can be seen from the side and detected twice (both papers).
    # The arrays are sorted by distance, so keep the first (closest) occurrence
    # of each ID and drop any later duplicates.
    if objectIDs is not None:
        seen = set()
        keep = []
        for i in range(len(objectIDs)):
            if objectIDs[i] in seen:
                continue
            seen.add(objectIDs[i])
            keep.append(i)
        objectIDs = objectIDs[keep]
        dists = dists[keep]
        angles = angles[keep]

    idx = 0
    for i in range(len(objectIDs)):
        if objectIDs[i] == 6:
            break
        else: idx += 1
## Drive towards L1

    ## Turn the amount of degrees we need
    
    angle = np.rad2deg(angles[idx])
    print("Angle is:", angle)
    drive.turn(angle)
    ## Drive the distance
    distance = dists[idx] / 100
    print("distance is :", distance)
    drive.drive(distance)





def main():
# Plan
## Start out by localising our selves. 
    
    drivetol1()




## Self localise





## RRt to L2

## Use some obstacle avoidance using Sonar

## Repeat from L2
### We know that sometimes 

main()
