from time import sleep

import robot


arlo = robot.Robot()

LEFTSPEED  = 69
RIGHTSPEED = 64
CIRCLETIME = 2.85
DRIVETIME  = 2.3
EPS        = 0.041

def turn(angle):
    left = 1
    right = 0
    if (angle < 0):
        left, right = right, left

    print(arlo.go_diff(LEFTSPEED, RIGHTSPEED,left, right))
    sleep((abs(angle)/360)*CIRCLETIME)
    print(arlo.stop())
    sleep(EPS)

def drive(length):
    print(arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1))
    sleep(DRIVETIME*length)
    print(arlo.stop())
    sleep(EPS)

turn(-360)

#drive(3)
