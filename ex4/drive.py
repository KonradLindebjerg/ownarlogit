from time import sleep

import robot


arlo = robot.Robot()

leftSpeed = 64
rightSpeed = 64
circleTime = 2.5   

def turn(angle):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(circleTime)
    print(arlo.stop())
turn(10)


