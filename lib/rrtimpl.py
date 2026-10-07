import os
from time import sleep

import math
from lib import drive
from lib import rrt_pi as rrt
#from lib import landmarkmapping as lm
from lib.netplot import PlotSender

# IP of the laptop running plot_client.py. Override with:  PLOT_HOST=<ip> python ex4-2.py
PLOT_HOST    = os.environ.get("PLOT_HOST", "172.20.10.3") # Hardcoded konrads ip
PLOT_PORT    = int(os.environ.get("PLOT_PORT", "5005"))
ROBOT_RADIUS = 0.225
EPS          = 0.5

def rotate_obstacles(obstacles, heading_deg):
    """Rotate camera-frame detections into the starting map frame.

    Camera frame: x = right, z = forward (this is what detectLandmark returns
    as (x, z, radius)). heading_deg is the robot's CCW-positive orientation
    relative to the start orientation. Because we only rotate in place, the
    robot stays at the origin, so no translation is needed.
    """
    phi = math.radians(heading_deg)
    cos_p, sin_p = math.cos(phi), math.sin(phi)
    rotated = []
    for (x, z, r) in obstacles:
        wx = x * cos_p - z * sin_p
        wy = x * sin_p + z * cos_p
        rotated.append((wx, wy, r))
    return rotated


#def scan_surroundings():
#    """Take 3 images to widen the field of view before planning.
#
#    Sequence: straight ahead, 45 degrees left, then 90 degrees right (ending
#    45 degrees right of start), then 45 degrees left back to the start
#    orientation. Detections from each pose are rotated into the starting map
#    frame and merged. drive.turn() is CW-positive, so a left (CCW) turn is
#    negated; `heading` tracks the CCW-positive orientation relative to start.
#    """
#    obstacles = []
#    heading = 0.0
#
#    # 1) Straight ahead.
#    obstacles += rotate_obstacles(lm.detectLandmark(), heading)
#    sleep(EPS)
#
#    # 2) Turn 45 degrees left (CCW).
#    drive.turn(-45)
#    sleep(EPS)
#    heading += 45
#    obstacles += rotate_obstacles(lm.detectLandmark(), heading)
#    sleep(EPS)
#
#    # 3) Turn 90 degrees right (CW) -> now 45 degrees right of start.
#    drive.turn(90)
#    sleep(EPS)
#    heading -= 90
#    obstacles += rotate_obstacles(lm.detectLandmark(), heading)
#    sleep(EPS)
#
#    # 4) Turn 45 degrees left to return to the start orientation.
#    drive.turn(-45)
#    sleep(EPS)
#    heading += 45
#
#    print("Scan complete, heading back at:", heading)
#    print("Detected obstacles (map frame):", obstacles)
#    return obstacles
#

def driveToGoal(robotrrt, path):
    print("Started driving to goal")
    i = len(path) - 2
    while (i >= 0):
        # Signed turn from the robot's current heading toward the next point
        angle = angle_to_target(robotrrt.position, robotrrt.robot_orientation, path[i])
        print("Driving to angle: ", angle)

        # Rotate angle on robot. `angle` is CCW-positive (math convention), but
        # drive.turn() is CW-positive (positive -> right turn), so negate it here.
        drive.turn(-angle)
        robotrrt.robot_orientation += angle
        # Drive distance on robot
        distance = calculate_drive_distance(robotrrt.position, path[i])
        print("Driving distance: ", distance)
        # Update robots position
        drive.drive(distance)
        robotrrt.position = path[i]
        i -= 1



def angle_to_target(position, orientation_deg, target):
    """Smallest signed rotation (degrees) to face `target` from `position`.

    Frame: the robot's forward axis is +y (the goal [0, 2] is straight ahead),
    orientation is measured in degrees, and a positive turn is counter-clockwise
    (heading measured from +y toward -x). This matches robot_orientation, so the
    caller can do `robot_orientation += angle` directly. drive.turn() uses the
    opposite (CW-positive) convention, so the caller negates when commanding it.
    """
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    desired_deg = math.degrees(math.atan2(-dx, dy))
    # Normalize to (-180, 180] so the robot always turns the short way.
    print((desired_deg - orientation_deg + 180) % 360 - 180)
    return (desired_deg - orientation_deg + 180) % 360 - 180

def calculate_drive_distance(u, v):
    lefthand  = (v[0] - u[0])**2
    righthand = (v[1] - u[1])**2
    return math.sqrt(lefthand + righthand)



def main(gx=0.0, gy=3.0):
    # Connect to the laptop's live plotter (headless-safe: runs anyway if it fails).
    sender = PlotSender(PLOT_HOST, PLOT_PORT)

    obstacleList = scan_surroundings()

    # Reachable region in meters: [xmin, xmax, ymin, ymax]. Must contain the
    # goal (e.g. y up to 2.0), or the tree can never reach it.
    play_area=[(-1.60 + ROBOT_RADIUS ), 1.10 - ROBOT_RADIUS , 0, 4]

    # rand_area is the sampling box, and it uses ONE [min, max] for both x and y.
    # Span the full play area so every reachable point (goal included) can be
    # sampled; nodes outside play_area are rejected anyway.
    rand_area = [min(play_area[0], play_area[2]), max(play_area[1], play_area[3])]

    # Set Initial parameters
    robotrrt = rrt.RRT(
        start=[0, 0 - ROBOT_RADIUS],
        goal=[gx, gy],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        play_area=play_area,
        robot_radius= ROBOT_RADIUS,
        plot_sender=sender,
    )

    path = robotrrt.planning(animation=True)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        # Push the final graph with the solution path to the laptop plot.
        robotrrt.draw_graph(path=path)

    print("Path is:")
    print(path)
    print("orientation is")
    print(robotrrt.robot_orientation)

    sleep(2)

    driveToGoal(robotrrt, path)


    sender.close()


    

    return path


if __name__ == "__main__":
    main()

