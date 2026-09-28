from time import sleep

import robot


arlo = robot.Robot()

leftSpeed  = 68
rightSpeed = 62
circleTime = 2.8
driveTime  = 2.2

def turn(angle):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(circleTime)
    print(arlo.stop())
turn(10)

def drive(length):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(driveTime)
    print(arlo.stop())


#drive(1)
