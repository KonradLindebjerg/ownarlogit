import os

import math
import drive
import rrt_pi as rrt
import landmarkmapping as lm
from netplot import PlotSender

# IP of the laptop running plot_client.py. Override with:  PLOT_HOST=<ip> python ex4-2.py
PLOT_HOST = os.environ.get("PLOT_HOST", "172.20.10.3") # Hardcoded konrads ip
PLOT_PORT = int(os.environ.get("PLOT_PORT", "5005"))

def driveToGoal(robotrrt, path):
    print("Started driving to goal")
    i = len(path) - 2
    while (i <= 0):
        # Calculate angle to next point
        angle = angle_between_vectors(robotrrt.position, path[i])
        # Rotate angle on robot
        drive.turn(angle)
        robotrrt.robot_orientation += angle
        # Drive distance on robot
        distance = calculate_drive_distance(robotrrt.position, path[i])
        # Update robots position
        drive.drive(distance)
        robotrrt.position = path[i]
        i -= 1


def angle_between_vectors(u, v):
    dot_product = sum(i*j for i, j in zip(u, v))
    norm_u = math.sqrt(sum(i**2 for i in u))
    norm_v = math.sqrt(sum(i**2 for i in v))
    cos_theta = dot_product / (norm_u * norm_v)
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def calculate_drive_distance(u, v):
    lefthand  = (v[0] - u[0])**2
    righthand = (v[1] - u[1])**2
    return math.sqrt(lefthand + righthand)



def main(gx=0.0, gy=2.0):
    # Connect to the laptop's live plotter (headless-safe: runs anyway if it fails).
    sender = PlotSender(PLOT_HOST, PLOT_PORT)

    obstacleList = lm.detectLandmark()

    # Set Initial parameters
    robotrrt = rrt.RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-1.14, 1.36],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.25,
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

    driveToGoal(robotrrt, path)


    sender.close()


    

    return path


if __name__ == "__main__":
    main()

