from time import sleep

import robot


# Create a robot object and initialize
arlo = robot.Robot()

outerSpeed = 90    # power of the faster (outer) wheel, in {0, [40;127]}
innerSpeed = 60    # power of the slower (inner) wheel, in {0, [40;127]}
circleTime = 5.6   # seconds for one full circle  <-- TUNE THIS FIRST
numEights  = 2     # how many figure-8s to drive

# Small delay so the Arduino has time to process each command.
CMD_WAIT = 0.041


def drive_circle(leftSpeed, rightSpeed):
    """Drive one full circle (both wheels forward) for circleTime seconds."""
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(circleTime)


for i in range(numEights):
    print("Figure-8 number", i + 1)

    # First loop: left wheel fast, right wheel slow -> curve to the RIGHT
    drive_circle(outerSpeed, innerSpeed)

    # Brief stop between the two loops for a cleaner transition
    print(arlo.stop())
    sleep(CMD_WAIT)

    # Second loop: right wheel fast, left wheel slow -> curve to the LEFT
    drive_circle(innerSpeed, outerSpeed)

    # Stop before the next figure-8
    print(arlo.stop())
    sleep(CMD_WAIT)


# Make sure the robot is stopped at the end
print(arlo.stop())
