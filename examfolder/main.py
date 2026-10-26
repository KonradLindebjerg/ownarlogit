import cv2
import math
import random
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
ROBOT_RADIUS = 25.0     # Robot radius for rrt
ARRIVED_RADIUS = 55.0   # cm from landmark centre that counts as arrived (40 cm + PF slack)
BACKUP_CM    = 50.0     # how far to reverse after the sonar stops us, before replanning
MAX_REPLANS  = 5        # max replans per landmark leg
VERIFY_RADIUS  = 65.0   # after a leg, we must be confidently within this many cm of the landmark
VERIFY_POS_STD = 25.0   # max particle spread (cm) for the position estimate to count as confident
# Non-target landmarks are keep-out zones so we never visit out of order / revisit.
# Box half-width (20) + visit distance (40) + safety margin (15). RRT adds ROBOT_RADIUS on top.
KEEPOUT_RADIUS = 20.0 + 40.0 + 15.0

# Landmarks and maps

DISTANCETOWALL = 53 


play_area = [-120 + DISTANCETOWALL + ROBOT_RADIUS, 520 - DISTANCETOWALL - ROBOT_RADIUS, 
             -120 + DISTANCETOWALL + ROBOT_RADIUS, 420 - DISTANCETOWALL - ROBOT_RADIUS] # Size of the map
rand_area = [min(play_area[0], play_area[2]), max(play_area[1], play_area[3])] # Sampling area for RRT

landmarkIDs = {
    "LANDMARK1": 8,
    "LANDMARK2": 1,
    "LANDMARK3": 9,
    "LANDMARK4": 11,
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



def turntowards(particles, x, y):
    """Turn the robot (shortest way) so its estimated heading points at map point (x, y) in cm.
    Particles are updated in place by drive.turn; also returned for convenience."""
    est_pose = particle.estimate_pose(particles)
    # Theta is CCW from +x (radians); drive.turn is CCW-positive (degrees).
    target_theta = math.atan2(y - est_pose.getY(), x - est_pose.getX())
    delta = target_theta - est_pose.getTheta()
    delta = math.atan2(math.sin(delta), math.cos(delta))  # wrap to [-pi, pi]
    print("Turning towards", (x, y), "by", math.degrees(delta), "deg")
    drive.turn(math.degrees(delta), particles)
    sleep(EPS)
    return particles


def landmark_keepouts(obstacleList, target_id, start, particles):
    """Drop scanned landmark markers from obstacleList and add every non-target
    landmark as a KEEPOUT_RADIUS obstacle at its map position.
    If the start is already inside a keep-out (e.g. the landmark we just visited),
    shrink it to just the physical box, so RRT is free to drive off in any
    direction without bumping into it. (Shrinking it only to the robot's current
    distance boxed the planner in: nearly every edge out of the start collided.)"""
    out = [o for o in obstacleList if o[3] not in LANDMARK_IDS]
    for lid, (lx, ly) in LANDMARK_COORDS.items():
        if lid == target_id:
            continue
        r = KEEPOUT_RADIUS
        d = math.hypot(start[0] - lx, start[1] - ly)
        if d <= r + ROBOT_RADIUS:
            r = min(lm.landmarkR, d - ROBOT_RADIUS - 2.0)
            if r <= 0.0:
                # Even a zero-radius obstacle blocks ROBOT_RADIUS around its centre,
                # so the start would be in collision and RRT could never plan.
                print(f"Start is right at landmark {lid}, leaving it out of the obstacles")
                continue
            drive.reverse(20, particles)
            print(f"Start is inside keep-out of landmark {lid}, shrinking it to {r:.1f} cm")
        out.append((lx, ly, r, lid))
    return out, particles


def rrttopoint(particles, x, y, target_id, attempt=0):
    # Face the goal before scanning, so the scan covers the direction we'll drive.
    particles = turntowards(particles, x, y)

    est_pose = particle.estimate_pose(particles)
    scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
    obstacleList = rrt.scan_surroundings(cam, scan_heading_deg , particles)
    # scan_surroundings only rotates detections; shift them to the robot's map position.
    obstacleList = [(ox + est_pose.getX(), oy + est_pose.getY(), r, oid)
                    for (ox, oy, r, oid) in obstacleList]

    est_pose = particle.estimate_pose(particles)
    obstacleList, particles = landmark_keepouts(obstacleList, target_id,
                                     (est_pose.getX(), est_pose.getY()), particles)
    print("Obstacles incl. landmark keep-outs:", obstacleList)

    robotrrt = rrt.RRT(
        start=[est_pose.getX(),est_pose.getY()],
        goal=[x, y],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        play_area=play_area,
        robot_radius= ROBOT_RADIUS,
        # RRT/angle_to_target measure heading from +y; particle theta is from +x.
        robot_orientation= math.degrees(est_pose.getTheta()) - 90.0,
        # Step/resolution in cm (map frame). The defaults (3.0 / 0.5) are
        # meter-scaled and far too fine for a ~400-500 cm map.
        path_resolution=5.0,
    )
    
    path = robotrrt.planning(animation=True)
    print("Path is:")
    print(path)

    
    path = shortcut_path_safe = rrt.shortcut_path(robotrrt, path)
    print("Optimized path is:")
    print(path)

    if path is None:
        # Goal is probably inside an obstacle: retry at a random point 40 cm away.
        if attempt >= 5:
            print("No path found after retries, giving up")
            return particles
        a = random.uniform(0, 2 * math.pi)
        new_x = x + 40 * math.cos(a)
        new_y = y + 40 * math.sin(a)
        print("No path, retrying at", new_x, new_y)
        return rrttopoint(particles, new_x, new_y, target_id, attempt + 1)
    
    
    print("orientation is")
    print(robotrrt.robot_orientation)
    
    if not rrt.driveToGoal(robotrrt, path, particles):
        # Blocked on the way to this waypoint. Back off; the caller replans anyway.
        print("Blocked on the way to waypoint, backing up")
        drive.reverse(BACKUP_CM, particles)
    return particles



def at_landmark(particles, landmarkcoordinates):
    """True if the particle filter puts us within ARRIVED_RADIUS of the landmark."""
    est_pose = particle.estimate_pose(particles)
    d = math.hypot(est_pose.getX() - landmarkcoordinates[0],
                   est_pose.getY() - landmarkcoordinates[1])
    print(f"Estimated distance to landmark: {d:.1f} cm")
    return d <= ARRIVED_RADIUS


def rrttolandmark(particles, landmarkID, landmarkcoordinates, observations=None):
    """observations: landmark IDs from a localize() we just did; pass it to skip localizing again."""
    newgoal = landmarkcoordinates 
    if observations is None:
        particles, observations = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)
    id, dists, angles, particles = findNextLandmark(landmarkID, particles)

    # Plan to the landmark if we're poorly localized OR can't see the target landmark.
    if len(observations) < 2 or id is None: 
        print("COULDNT FIND TARGET LANDMARK OR TWO OR LESS OBSERVED, GOING HALFWAY")
        # Halfway between where we are now and the goal (not between origin and goal).
        est_pose = particle.estimate_pose(particles)
        mid_x = (est_pose.getX() + newgoal[0]) / 2
        mid_y = (est_pose.getY() + newgoal[1]) / 2
        particles = rrttopoint(particles, mid_x, mid_y, landmarkID)
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
    
    # Default to the map position (box center); use the observed marker if we see it.
    goalx, goaly = newgoal[0], newgoal[1]
    edge = lm.landmarkR  # distance from box center to its side
    length = len(obstacleList)
    i = 0
    while i < length:
        if obstacleList[i][3] == landmarkID:
            # The marker sits on the box side facing us, so it already is the edge.
            goalx, goaly = obstacleList[i][0], obstacleList[i][1]
            edge = 0.0
            obstacleList.pop(i)
            i -= 1
            length -= 1
        i += 1

    # A visit counts within 40 cm of the box side, so stop with the robot's
    stop = edge + 10
    dx = est_pose.getX() - goalx
    dy = est_pose.getY() - goaly
    d = math.hypot(dx, dy)
    if d <= stop:
        print("Already within reach of landmark", landmarkID)
        return particles
    goalx += dx / d * stop
    goaly += dy / d * stop
    print("Landmark goal set to", goalx, goaly)

    obstacleList, particles = landmark_keepouts(obstacleList, landmarkID,
                                     (est_pose.getX(), est_pose.getY()), particles)
    print("Obstacles incl. landmark keep-outs:", obstacleList)

    #maybe change something with the heading or modify rrt so that it also uses heading
    robotrrt = rrt.RRT(
        start=[est_pose.getX(),est_pose.getY()],
        goal=[goalx, goaly],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        play_area=play_area,
        robot_radius= ROBOT_RADIUS,
        # RRT/angle_to_target measure heading from +y; particle theta is from +x.
        robot_orientation= math.degrees(est_pose.getTheta()) - 90.0,
        # Step/resolution in cm (map frame). The defaults (3.0 / 0.5) are
        # meter-scaled and far too fine for a ~400-500 cm map.
        path_resolution=5.0,
    )
    
    path = robotrrt.planning(animation=True)
    if path is None:
        return rrttolandmark(particles, landmarkID, landmarkcoordinates)
    else:
        print("Path is:")
        print(path)

        
        path = shortcut_path_safe = rrt.shortcut_path(robotrrt, path)
        print("Optimized path is:")
        print(path)
        
        
        print("orientation is")
        print(robotrrt.robot_orientation)
        
        # SET GOAL TO L2
        
        if rrt.driveToGoal(robotrrt, path, particles):
            return particles

        # The front sonar stopped us early. If that's the landmark box itself, we're done.
        particles, _, _, _, _ = sl.observe(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)
        if at_landmark(particles, landmarkcoordinates):
            print("Stopped early but already at landmark", landmarkID)
            return particles
        # Otherwise an obstacle is in the way: back off and plan again.
        print("Blocked before landmark", landmarkID, "- backing up and replanning")
        drive.reverse(BACKUP_CM, particles)
        return rrttolandmark(particles, landmarkID, landmarkcoordinates)


def gotolandmark(particles, landmarkID, landmarkcoordinates):
    """RRT to the landmark, then re-localize and check we're confidently within
    VERIFY_RADIUS of it. If not, RRT again (up to MAX_REPLANS times)."""
    observations = None
    for attempt in range(MAX_REPLANS + 1):
        particles = rrttolandmark(particles, landmarkID, landmarkcoordinates, observations)
        particles, observations = sl.localize(cam, LANDMARK_IDS, LANDMARK_COORDS, particles)

        pos_std, _ = sl.filter_confidence(particles)
        est_pose = particle.estimate_pose(particles)
        d = math.hypot(est_pose.getX() - landmarkcoordinates[0],
                       est_pose.getY() - landmarkcoordinates[1])
        print(f"Verify landmark {landmarkID}: distance {d:.1f} cm, spread {pos_std:.1f} cm")
        if d <= VERIFY_RADIUS and pos_std <= VERIFY_POS_STD:
            print("Confidently at landmark", landmarkID)
            return particles
        print(f"Not confidently within {VERIFY_RADIUS} cm of landmark {landmarkID}, replanning "
              f"(attempt {attempt + 1}/{MAX_REPLANS})")

    print("Gave up verifying landmark", landmarkID, "- moving on")
    return particles


def drivetol1(particles):
    #TODO DONT ASSUME WE CAN GO STRAIGHT TO THE LANDMARK
    _, dist, angle, particles = findNextLandmark(landmarkIDs["LANDMARK1"], particles)
    if angle is None:
        particles = drivetol1(particles)
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

    particles = gotolandmark(particles, landmarkIDs["LANDMARK2"], landmarks["LANDMARK2"])

    
    # DRIVE TO L3
    ## Start out by localizing itself
    #scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0

    particles = gotolandmark(particles, landmarkIDs["LANDMARK3"], landmarks["LANDMARK3"])

    # DRIVE TO L4
    ## Start out by localizing itself
    #scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
    particles = gotolandmark(particles, landmarkIDs["LANDMARK4"], landmarks["LANDMARK4"])

    # DRIVE BACK TO L1
    ## Start out by localizing itself
    #if all(VisitedLandmark.values()):
    #scan_heading_deg = math.degrees(est_pose.getTheta()) - 90.0
    particles = gotolandmark(particles, landmarkIDs["LANDMARK1"], landmarks["LANDMARK1"])




main()
