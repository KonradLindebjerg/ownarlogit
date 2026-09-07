from time import sleep

import robot


arlo = robot.Robot()

leftSpeed = 64
rightSpeed = 64
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))


# Wait a bit while robot moves forward
sleep(3)

sleep(0.041)


print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))
sleep(3)

sleep(0.041)


# send a stop command
print(arlo.stop())

# send a go_diff command to drive forward in a curve turning right
leftSpeed = 64
rightSpeed = 32
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

# Wait a bit while robot moves forward
sleep(3)

# send a stop command
print(arlo.stop())



# send a go_diff command to drive forward in a curve turning right
leftSpeed = 64
rightSpeed = 32
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

# Wait a bit while robot moves forward
sleep(3)

# send a stop command
print(arlo.stop())

