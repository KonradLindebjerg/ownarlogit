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
EPS          = 0.075    # EPS for sleep time so arlo hsa time to catch up
SIGMA_THETA  = 0.029320 # Sigma noise for when turning
SIGMA        = 2.0      # Sigma for driving distance
ROBOT_RADIUS = 22.5     # Robot radius for rrt

# Landmarks and maps

DISTANCETOWALL = 53 


play_area = [-120 + DISTANCETOWALL + ROBOT_RADIUS, 620 - DISTANCETOWALL - ROBOT_RADIUS, 
             -120 + DISTANCETOWALL + ROBOT_RADIUS, 520 - DISTANCETOWALL - ROBOT_RADIUS] # Size of the map
rand_area = [min(play_area[0], play_area[2]), max(play_area[1], play_area[3])] # Sampling area for RRT

landmarkIDs = {
    "LANDMARK1": 5,
    "LANDMARK2": 3,
    "LANDMARK3": 7,
    "LANDMARK4": 10,
}
## Landmark Coordinates
landmarks = {
    "LANDMARK1": (0.0, 0.0),
    "LANDMARK2": (0.0, 300.0),
    "LANDMARK3": (400.0, 0.0),
    "LANDMARK4": (400.0, 300.0),
}

LANDMARK_IDS    = set(landmarkIDs.values())
LANDMARK_COORDS = {landmarkIDs[name]: landmarks[name] for name in landmarkIDs}

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


### HELPERS ###

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



def goal_blocked(x, y, obstacles, margin=5.0):
    """True if the robot can't stand at (x, y) without hitting an obstacle."""
    return any(math.hypot(x - ox, y - oy) <= r + ROBOT_RADIUS + margin
               for (ox, oy, r, _) in obstacles)


def alternative_goals(x, y, obstacles, start, rings=(30, 50, 75), n_angles=8):
    """Free points on rings around (x, y), best first.

    Prefers points close to the original goal, tie-breaking toward the robot
    (shorter drive, and less likely to be hidden behind the blocking box).
    """
    cands = []
    for rad in rings:
        for k in range(n_angles):
            a = 2 * math.pi * k / n_angles
            cx, cy = x + rad * math.cos(a), y + rad * math.sin(a)
            if not (play_area[0] <= cx <= play_area[1] and
                    play_area[2] <= cy <= play_area[3]):
                continue
            if goal_blocked(cx, cy, obstacles):
                continue
            score = rad + 0.2 * math.hypot(cx - start[0], cy - start[1])
            cands.append((score, cx, cy))
    cands.sort()
    return [(cx, cy) for _, cx, cy in cands]


def plan_path(est_pose, x, y, obstacles, rings=(30, 50, 75),
              max_goals=6, tries_per_goal=2):
    """Plan an RRT path to (x, y), or to a free point near it if that fails.

    Returns (robotrrt, path), or (None, None) if nothing reachable was found.
    """
    start = [est_pose.getX(), est_pose.getY()]

    goals = []
    if goal_blocked(x, y, obstacles):
        print("Goal (%.0f, %.0f) is inside an obstacle, trying nearby points" % (x, y))
    else:
        goals.append((x, y))
    goals += alternative_goals(x, y, obstacles, start, rings=rings)

    for gx, gy in goals[:max_goals]:
        # RRT is random, so one failure on a free goal can just be bad luck.
        for _ in range(tries_per_goal):
            robotrrt = rrt.RRT(
                start=start,
                goal=[gx, gy],
                rand_area=rand_area,
                obstacle_list=obstacles,
                play_area=play_area,
                robot_radius=ROBOT_RADIUS,
                # RRT/angle_to_target measure heading from +y; particle theta is from +x.
                robot_orientation=math.degrees(est_pose.getTheta()) - 90.0,
                # Step/resolution in cm (map frame). The defaults (3.0 / 0.5) are
                # meter-scaled and far too fine for a ~400-500 cm map.
                path_resolution=5.0,
            )
            path = robotrrt.planning(animation=True)
            if path is not None:
                print("Planned to (%.0f, %.0f), path is:" % (gx, gy))
                print(path)
                path = rrt.shortcut_path(robotrrt, path)
                print("Optimized path is:")
                print(path)
                return robotrrt, path
        print("No path to (%.0f, %.0f)" % (gx, gy))

    print("No reachable point near (%.0f, %.0f)" % (x, y))
    return None, None


def rrttopoint(particles, x, y):
    est_pose = particle.estimate_pose(particles)

    # Face the goal before scanning, so the scan covers the direction we'll drive.
    # Theta is CCW from +x (radians); drive.turn is CCW-positive (degrees).
    target_theta = math.atan2(y - est_pose.getY(), x - est_pose.getX())
    delta = target_theta - est_pose.getTheta()
    delta = math.atan2(math.sin(delta), math.cos(delta))  # wrap to [-pi, pi]
    drive.turn(math.degrees(delta), particles)
    sleep(EPS)

    est_pose = particle.estimate_pose(particles)
    scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
    obstacleList = rrt.scan_surroundings(cam, scan_heading_deg , particles)
    # scan_surroundings only rotates detections; shift them to the robot's map position.
    obstacleList = [(ox + est_pose.getX(), oy + est_pose.getY(), r, oid)
                    for (ox, oy, r, oid) in obstacleList]

    est_pose = particle.estimate_pose(particles)

    robotrrt, path = plan_path(est_pose, x, y, obstacleList)
    if path is None:
        # Leave it to the caller to relocalize / rescan and try again.
        return particles

    print("orientation is")
    print(robotrrt.robot_orientation)
    
    rrt.driveToGoal(robotrrt, path, particles)
    return particles



def rrttolandmark(particles, est_pose, landmarkID,landmarkcoordinates ):
    newgoal = landmarkcoordinates 
    particles, observations = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)

    # Plan to the landmark if we're poorly localized OR can't see the target landmark.
    if (observations and len(observations) < 2) or landmarkID not in observations:
        print("COULDNT FIND LANDMARK OR LESS THAN TWO OBSERVED, GOING HALFWAY")
        # Halfway between where we are now and the goal (not between origin and goal).
        est_pose = particle.estimate_pose(particles)
        mid_x = (est_pose.getX() + newgoal[0]) / 2
        mid_y = (est_pose.getY() + newgoal[1]) / 2
        particles = rrttopoint(particles, mid_x, mid_y)
        particles, _ = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)

    id, dists, angles, particles = findNextLandmark(landmarkID, particles)
    est_pose = particle.estimate_pose(particles)
    scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
    obstacleList = rrt.scan_surroundings(cam, scan_heading_deg , particles)
    # scan_surroundings only rotates detections; shift them to the robot's map position.
    obstacleList = [(x + est_pose.getX(), y + est_pose.getY(), r, oid)
                    for (x, y, r, oid) in obstacleList]
    print("LINE 153: Obstaclelist is: ", obstacleList)
    est_pose = particle.estimate_pose(particles)
    #x,y, radius, idss = obstacleList
    
    length = len(obstacleList)
    i = 0
    while i < length:
        if obstacleList[i][3] == landmarkID:
            #goalx, goaly = obstacleList[i][0], obstacleList[i][1]
            obstacleList.pop(i)
            i -= 1
            length -= 1
        i += 1
            
    
    #maybe change something with the heading or modify rrt so that it also uses heading
    # Only small rings here: we must end within 40 cm of the landmark to count.
    robotrrt, path = plan_path(est_pose, newgoal[0], newgoal[1], obstacleList,
                               rings=(20, 30))
    if path is None:
        return particles

    print("orientation is")
    print(robotrrt.robot_orientation)
    
    # SET GOAL TO L2
    
    rrt.driveToGoal(robotrrt, path, particles)
    return particles


def drivetol1(particles):
    #TODO DONT ASSUME WE CAN GO STRAIGHT TO THE LANDMARK
    _, dist, angle, particles = findNextLandmark(landmarkIDs["LANDMARK1"], particles)
    if angle is None:
        drivetol1(particles)
        print("L1 not found, retrying")
        return particles

    ## Turn towards L1. Camera angle is positive-left, drive.turn is positive-left.
    angle = np.rad2deg(angle)
    print("Angle is:", angle)
    drive.turn(angle, particles)
    particles, _, _, _, _ = sl.observe(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)

    ## Drive the distance
    print("distance is :", dist)
    drive.drive(dist - 10, particles)
    particles, _, _, _, _ = sl.observe(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)

    VisitedLandmark["LANDMARK1"] = True
    return particles


def searchState():
    return


def findNextLandmark(targetLandmark, particles):
    """Turn in sl.SEARCH_TURN_STEP_DEG steps (one full turn at most) until
    targetLandmark is seen, updating the particle filter on every frame so the
    turns don't silently corrupt the pose.
    Returns (id, dist, angle, particles); id/dist/angle are None if not found."""
    for step in range(sl.SEARCH_STEPS):
        print(step)
        particles, objectIDs, dists, angles, observations = sl.observe(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)
        objectIDs, dists, angles = deduplicate(objectIDs, dists, angles)

        if objectIDs is not None:
            for i in range(len(objectIDs)):
                if objectIDs[i] == targetLandmark:
                    print("FOUND TARGET LANDMARK: ", targetLandmark)
                    return objectIDs[i], dists[i], angles[i], particles
        drive.turn(sl.SEARCH_TURN_STEP_DEG, particles)
        sleep(EPS)

    return None, None, None, particles


def main():
# Plan
## Start out by localising our selves.
    num_particles = 1000
    particles = sl.initialize_particles(num_particles)
    #est pose og particles, spørgsmålet om vi måske skal bruge mindst 2 før vi kan være sikker på localize
    #måske localize skal ændres til at bruge 2? eller skal vi have en seperat funktion til starten? idk 




    particles, _ = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)

    particles = drivetol1(particles)
    #we should now have an idea of where we are?
    est_pose = particle.estimate_pose(particles)
    
## Self localise
    #TODO DONT ASSUME THE BELOW
    # We are now at Landmark 1 and now begin ussing RRT to go to the next landmark

    # We assume that we find the next landmark here
    # Reobserve our surroundings
    #RRT to landmark 2
    particles = rrttolandmark(particles, est_pose, landmarkIDs["LANDMARK2"], landmarks["LANDMARK2"])
    
    # DRIVE TO L3
    ## Start out by localizing itself
    particles, _ = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)
    id, dists, angles, particles = findNextLandmark(landmarkIDs["LANDMARK3"], particles)
    est_pose = particle.estimate_pose(particles)
    #scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0

    particles = rrttolandmark(particles, est_pose, landmarkIDs["LANDMARK3"], landmarks["LANDMARK3"])

    # DRIVE TO L4
    ## Start out by localizing itself
    particles, _ = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)
    id, dists, angles, particles = findNextLandmark(landmarkIDs["LANDMARK4"], particles)
    est_pose = particle.estimate_pose(particles)
    #scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
    particles = rrttolandmark(particles, est_pose, landmarkIDs["LANDMARK4"], landmarks["LANDMARK4"])

    # DRIVE BACK TO L1
    ## Start out by localizing itself
    if all(VisitedLandmark.values()):
        particles, _  = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)
        id, dists, angles, particles = findNextLandmark(landmarkIDs["LANDMARK1"], particles)
        est_pose = particle.estimate_pose(particles)
        #scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
        rrttolandmark(particles, est_pose, landmarkIDs["LANDMARK1"],landmarks["LANDMARK1"] )

main()
