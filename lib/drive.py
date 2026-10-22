from time import sleep
import sys 
import os
import math



import numpy as np

import robot


sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from examfolder import particle 

arlo = robot.Robot()


leftSpeed           = 69
rightSpeed          = 67
circleTime          = 2.95
driveTime           = 2.25
eps                 = 0.3

SIGMA               = 0.02     # Sigma for driving distance
SIGMA_THETA         = 0.029320 # Sigma noise for when turning (process/motion heading noise)
ROUGHEN_SIGMA       = 4.0      # cm
ROUGHEN_SIGMA_THETA = 0.05     # rad (~2.9 deg)

def turn(angle, particles):
    left  = 0
    right = 1
    if (angle < 0):
        left, right = right, left

    print(arlo.go_diff(leftSpeed, rightSpeed,left, right))
    sleep((abs(angle)/360)*circleTime)
    print(arlo.stop())
    sleep(eps)
    delta_theta = np.deg2rad(angle)
    for p in particles:
        particle.move_particle(p, 0, 0, delta_theta)
    theta_noise = max(SIGMA_THETA * abs(delta_theta) / (2 * np.pi), 0.5 * ROUGHEN_SIGMA_THETA)
    particle.add_uncertainty(particles, 0.0, theta_noise)

    #maybe return particle but they get modified directly
    
    

def drive(length, particles):
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(driveTime*length/100)
    print(arlo.stop())
    sleep(eps)
    for p in particles:
        particle.move_particle(p,
                        length * np.cos(p.getTheta()),
                        length * np.sin(p.getTheta()),
                        0.0)
    sigma_noise = SIGMA*length
    particle.add_uncertainty(particles, sigma_noise, 0.0)
    
    
    


def angle_to_target(position, orientation_deg, target):
    """Smallest signed rotation (degrees) to face `target` from `position`.

    Frame: the particle-filter convention. Forward axis is +x (theta == 0 points
    along +x), orientation is measured in degrees, and a positive turn is
    counter-clockwise (heading measured from +x toward +y). This matches the
    particles' theta (whose motion is d*cos(theta), d*sin(theta)), so the caller
    passes est_pose.getTheta() (converted to degrees) directly as orientation.
    drive.turn() uses the opposite (CW-positive) convention, so the caller
    negates when commanding it: drive.turn(-angle).
    """
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    desired_deg = math.degrees(math.atan2(dy, dx))
    # Normalize to (-180, 180] so the robot always turns the short way.
    print((desired_deg - orientation_deg + 180) % 360 - 180)
    return (desired_deg - orientation_deg + 180) % 360 - 180

def calculate_drive_distance(u, v):
    lefthand  = (v[0] - u[0])**2
    righthand = (v[1] - u[1])**2
    return math.sqrt(lefthand + righthand)



if __name__ == "__main__":
    turn(360)
    drive(200)



