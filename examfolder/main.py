import cv2
import math
import camera
import numpy as np
import time
import rrt_pi as rrt
import landmarkmapping as lm
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
EPS         = 0.075    # EPS for sleep time so arlo hsa time to catch up
SIGMA_THETA = 0.029320 # Sigma noise for when turning
SIGMA       = 2.0      # Sigma for driving distance
ROBOT_RADIUS = 22.5   # Robot radius for rrt

# Landmarks and maps
play_area = [-120 + ROBOT_RADIUS, 620 - ROBOT_RADIUS, 
             -120 + ROBOT_RADIUS, 520 - ROBOT_RADIUS] # Size of the map
rand_area = [min(play_area[0], play_area[2]), max(play_area[1], play_area[3])] # Sampling area for RRT
landmarkIDs = [1,2,3,4]
## Landmark Coordinates
landmarks = {
    1: (0.0, 0.0),
    2: (0.0, 300.0),
    3: (400.0, 0.0),
    4: (400.0, 300.0),
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


def findNextLandmark(targetLandmark):

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
    #TODO DONT ASSUME THE BELOW
    # We are now at Landmark 1 and now begin ussing RRT to go to the next landmark

    # We assume that we find the next landmark here

    id, dists, angles = findNextLandmark(3)
    obstacleList = rrt.scan_surroundings(cam, angles)
    idss, tvecs, radius = obstacleList
    goalx, goaly= 0.0, 300.0

    for i in range(len(idss)):
        if idss[i] == 3:
            coords = tvecs[i]
            goalx, goaly = coords[1], coords[0]
            


    robotrrt = rrt.RRT(
        start=[0, 0 - ROBOT_RADIUS],
        goal=[goalx, goaly],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        play_area=play_area,
        robot_radius= ROBOT_RADIUS,
        # Step/resolution in cm (map frame). The defaults (3.0 / 0.5) are
        # meter-scaled and far too fine for a ~400-500 cm map.
        path_resolution=5.0,
    )

    path = robotrrt.planning(animation=True)
    print("GOAL IS", goalx, goaly)
    
    print("Path is:")
    print(path)
    print("orientation is")
    print(robotrrt.robot_orientation)

    # SET GOAL TO L2

    rrt.driveToGoal(robotrrt, path)



## RRt to L2

## Use some obstacle avoidance using Sonar

## Repeat from L2
### We know that sometimes 

main()
