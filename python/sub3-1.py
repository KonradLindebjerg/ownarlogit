from time import sleep

import robot

#test
# Create a robot object and initialize
arlo = robot.Robot()


dangerLimit = 300
sideLimit = 65
CMD_WAIT = 0.041



outerSpeed = 91    # power of the faster (outer) wheel, in {0, [40;127]}
innerSpeed = 51    # power of the slower (inner) wheel, in {0, [40;127]}
leftSpeed = 68
rightSpeed = 65



def decide_turn():
    frontSensor = arlo.read_front_ping_sensor()
    leftSensor = arlo.read_left_ping_sensor()
    backSensor = arlo.read_back_ping_sensor()
    rightSensor = arlo.read_right_ping_sensor()

    while (leftSensor >= sideLimit and rightSensor >= sideLimit and frontSensor >= dangerLimit):
        print(arlo.stop())
        print("front sensor: ", arlo.read_front_ping_sensor())
        print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
        frontSensor = arlo.read_front_ping_sensor()
        print(frontSensor)
        leftSensor = arlo.read_left_ping_sensor()
        backSensor = arlo.read_back_ping_sensor()
        rightSensor = arlo.read_right_ping_sensor()
    sleep(CMD_WAIT)
    print(arlo.stop())
    if (rightSensor>= dangerLimit):
        print(arlo.go_diff(leftSpeed,rightSpeed, 1, 0))
        sleep(0.2)
        print(arlo.stop())
        return
    if (backSensor >= dangerLimit):
        print(arlo.go_diff(leftSpeed,rightSpeed, 0, 1))
        sleep(0.2)
        print(arlo.stop())
        return








while (True):
    #print("front sensor: ", arlo.read_front_ping_sensor())
    #print("right sensor: ", arlo.read_right_ping_sensor())
    #print("left sensor: ", arlo.read_left_ping_sensor())
    #print("back sensor: ", arlo.read_back_ping_sensor())
    #sleep(3)
    decide_turn()




print(arlo.stop())
