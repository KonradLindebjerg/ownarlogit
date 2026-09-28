from time import sleep

import robot


arlo = robot.Robot()

leftSpeed = 64
rightSpeed = 64
circleTime = 5.6   

def turn(angle):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(circleTime)
    print(arlo.stop())



