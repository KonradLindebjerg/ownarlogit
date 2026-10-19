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

cam = camera.Camera(0, robottype='arlo', useCaptureThread=False) # Initalise camera
sleep(EPS)

# We know that obstacle landmarks are > 4






def deduplicate(objectIDs, dists, angles):
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
        return objectIDs, dists, angles
    return None, None, None





def drivetol1():
    colour = cam.get_next_frame() # Capture image
    sleep(EPS)
    objectIDs, dists, angles = cam.detect_aruco_objects(colour) # Return ID's
    objectIDs, dists, angles = deduplicate(objectIDs, dists, angles)
    sleep(EPS)

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
    distance = dists[idx] 
    print("distance is :", distance)
    drive.drive(distance - 10)

    VisitedLandmark[1] = True


def searchState():
    return


def do360(targetLandmark):

    for i in range(18):
        print(i)

        colour = cam.get_next_frame() # Capture image
        sleep(EPS)
        objectIDs, dists, angles = cam.detect_aruco_objects(colour) # Return ID's
        sleep(EPS)
        objectIDs, dists, angles = deduplicate(objectIDs, dists, angles)
        
        if objectIDs is not None:
            for i in range(len(objectIDs)):
                if objectIDs[i] == targetLandmark:
                    print("FOUND TARGET LANDMARK: ", targetLandmark)
                    return objectIDs[i], dists[i], angles[i]
        drive.turn(20)
        sleep(EPS)

    return None, None, None


def main():
# Plan
## Start out by localising our selves. 
    
    drivetol1()

## Self localise

    do360(3)





## RRt to L2

## Use some obstacle avoidance using Sonar

## Repeat from L2
### We know that sometimes 

main()
