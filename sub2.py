from time import sleep

import robot


arlo = robot.Robot()

leftSpeed = 68
rightSpeed = 32
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

sleep(3)

sleep(0.041)

