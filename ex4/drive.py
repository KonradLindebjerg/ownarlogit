from time import sleep

import robot


arlo = robot.Robot()

leftSpeed  = 68
rightSpeed = 62
circleTime = 2.5
driveTime  = 2.0

def turn(angle):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(circleTime)
    print(arlo.stop())
#turn(10)

def drive(length):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(4)
    print(arlo.stop())


drive(1)
