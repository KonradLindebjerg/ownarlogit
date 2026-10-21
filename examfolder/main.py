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
import selflocalize as sl
import particle



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

landmarkIDs = {
    "LANDMARK1": 1,
    "LANDMARK2": 3,
    "LANDMARK3": 2,
    "LANDMARK4": 4,
}
## Landmark Coordinates
landmarks = {
    "LANDMARK1": (0.0, 0.0),
    "LANDMARK2": (0.0, 300.0),
    "LANDMARK3": (400.0, 0.0),
    "LANDMARK4": (400.0, 300.0),
}

## What landmarks have been visited?
VisitedLandmark = {
    "LANDMARK1": False,
    "LANDMARK2": False,
    "LANDMARK3": False,
    "LANDMARK4": False
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





def drivetol1(particles):
    #TODO DONT ASSUME WE CAN GO STRAIGHT TO THE LANDMARK
    colour = cam.get_next_frame() # Capture image
    sleep(EPS)
    objectIDs, dists, angles = cam.detect_aruco_objects(colour) # Return ID's
    objectIDs, dists, angles = deduplicate(objectIDs, dists, angles)
    sleep(EPS)

    idx = 0
    for i in range(len(objectIDs)):
        if objectIDs[i] == landmarkIDs["LANDMARK1"]:
            break
        else: idx += 1
## Drive towards L1

    ## Turn the amount of degrees we need
    
    angle = np.rad2deg(angles[idx])
    print("Angle is:", angle)
    drive.turn(angle, particles)
    ## Drive the distance
    distance = dists[idx] 
    print("distance is :", distance)
    drive.drive(distance - 10, particles)
 

    VisitedLandmark["LANDMARK1"] = True


def searchState():
    return


def findNextLandmark(targetLandmark, est_pose, particles):

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
                    return objectIDs[i], dists[i], angles[i], est_pose
        drive.turn(-20, particles)
        est_pose.setTheta(est_pose.getTheta() + -20) 
        sleep(EPS)

    return None, None, None, est_pose


def main():
# Plan
## Start out by localising our selves.
    num_particles = 1000
    particles = sl.initialize_particles(1000)
    #est pose og particles, spørgsmålet om vi måske skal bruge mindst 2 før vi kan være sikker på localize
    #måske localize skal ændres til at bruge 2? eller skal vi have en seperat funktion til starten? idk 
    est_pose, particles = sl.localize(cam, landmarkIDs, landmarks, particles)
    

    drivetol1(particles)
    #we should now have an idea of where we are?
    est_pose = particle.estimate_pose(particles)
    print("ESTIMATED POSE", est_pose.getX(), est_pose.getY(), est_pose.getTheta())
    
## Self localise
    #TODO DONT ASSUME THE BELOW
    # We are now at Landmark 1 and now begin ussing RRT to go to the next landmark

    # We assume that we find the next landmark here
    #rotated estimated heading by whatever we rotated
    id, dists, angles, est_pose = findNextLandmark(landmarkIDs["LANDMARK2"], est_pose, particles)
    obstacleList = rrt.scan_surroundings(cam, angles)
    print("LINE 153: Obstaclelist is: ", obstacleList)
    #x,y, radius, idss = obstacleList
    goalx, goaly= 0.0, 300.0

    length = len(obstacleList)
    i = 0
    while i < length:
        if obstacleList[i][3] == landmarkIDs["LANDMARK2"]:
            #goalx, goaly = obstacleList[i][0], obstacleList[i][1]
            obstacleList.pop(i)
            i -= 1
            length -= 1
        i += 1
            

    #maybe change something with the heading or modify rrt so that it also uses heading
    robotrrt = rrt.RRT(
        start=[est_pose.getX(),est_pose.getY()],
        goal=[goalx, goaly],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        play_area=play_area,
        robot_radius= ROBOT_RADIUS,
        robot_orientation= est_pose.getTheta(),
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
