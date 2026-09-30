import os
from time import sleep

import math
import drive
import rrt_pi as rrt
import landmarkmapping as lm
from netplot import PlotSender

# IP of the laptop running plot_client.py. Override with:  PLOT_HOST=<ip> python ex4-2.py
PLOT_HOST    = os.environ.get("PLOT_HOST", "172.20.10.3") # Hardcoded konrads ip
PLOT_PORT    = int(os.environ.get("PLOT_PORT", "5005"))
ROBOT_RADIUS = 0.250

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



def main(gx=0.0, gy=2.0):
    # Connect to the laptop's live plotter (headless-safe: runs anyway if it fails).
    sender = PlotSender(PLOT_HOST, PLOT_PORT)

    obstacleList = lm.detectLandmark()

    # Reachable region in meters: [xmin, xmax, ymin, ymax]. Must contain the
    # goal (e.g. y up to 2.0), or the tree can never reach it.
    play_area=[-1.15 - ROBOT_RADIUS , 1.15 - ROBOT_RADIUS , 0, 4]

    # rand_area is the sampling box, and it uses ONE [min, max] for both x and y.
    # Span the full play area so every reachable point (goal included) can be
    # sampled; nodes outside play_area are rejected anyway.
    rand_area = [min(play_area[0], play_area[2]), max(play_area[1], play_area[3])]

    # Set Initial parameters
    robotrrt = rrt.RRT(
        start=[0, 0],
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

