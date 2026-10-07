from time import sleep
import math

import robot


arlo = robot.Robot()


leftSpeed  = 68
rightSpeed = 65
circleTime = 2.8
driveTime  = 2.25
eps        = 0.1

def turn(angle):
    left = 1
    right = 0
    if (angle < 0):
        left, right = right, left

    print(arlo.go_diff(leftSpeed, rightSpeed,left, right))
    sleep((abs(angle)/360)*circleTime)
    print(arlo.stop())
    sleep(eps)

def drive(length):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(driveTime*length)
    print(arlo.stop())
    sleep(eps)


def angle_to_target(position, orientation_deg, target):
    """Smallest signed rotation (degrees) to face `target` from `position`.

    Frame: the particle-filter convention. Forward axis is +x (theta == 0 points
    along +x), orientation is measured in degrees, and a positive turn is
    counter-clockwise (heading measured from +x toward +y). This matches the
    particles' theta (whose motion is d*cos(theta), d*sin(theta)), so the caller
    passes est_pose.getTheta() (converted to degrees) directly as orientation.
    drive.turn() uses the opposite (CW-positive) convention, so the caller
    negates when commanding it: drive.turn(-angle).
    """
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    desired_deg = math.degrees(math.atan2(dy, dx))
    # Normalize to (-180, 180] so the robot always turns the short way.
    print((desired_deg - orientation_deg + 180) % 360 - 180)
    return (desired_deg - orientation_deg + 180) % 360 - 180

def calculate_drive_distance(u, v):
    lefthand  = (v[0] - u[0])**2
    righthand = (v[1] - u[1])**2
    return math.sqrt(lefthand + righthand)
