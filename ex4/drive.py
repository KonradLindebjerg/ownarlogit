from time import sleep

import robot


arlo = robot.Robot()

leftSpeed  = 68
rightSpeed = 62
circleTime = 2.75
driveTime  = 2.2

def turn(angle):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep((angle/360)*circleTime)
    print(arlo.stop())
turn(-90)

def drive(length):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(driveTime)
    print(arlo.stop())


#drive(1)
