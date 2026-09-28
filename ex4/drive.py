from time import sleep

import robot


arlo = robot.Robot()

leftSpeed  = 68
rightSpeed = 62
circleTime = 2.80
driveTime  = 2.2
eps        = 0.041

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

turn(-360)




