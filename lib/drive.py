import time
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
rightSpeed          = 66
circleTime          = 2.79
driveTime           = 2.25
eps                 = 0.3

SIGMA               = 0.02     # Sigma for driving distance
SIGMA_THETA         = 0.029320 # Sigma noise for when turning (process/motion heading noise)
ROUGHEN_SIGMA       = 4.0      # cm
ROUGHEN_SIGMA_THETA = 0.05     # rad (~2.9 deg)
TURN_NOISE_FRAC     = 0.15     # heading noise std as a fraction of the commanded turn

# Sonar safety stop while driving (cm). Tune on the robot.
STOP_CM             = 20.0     # stop if the front (or back, when reversing) sonar reads closer than this
# The "left"/"right" sonars point ~40-45 deg off the front, not straight sideways.
# They see boxes we're just driving past, so they only stop us when something is
# much closer (at 45 deg, a 15 cm reading is ~11 cm sideways from the sensor).
DIAG_STOP_CM        = 10.0
SONAR_POLL_DT       = 0.02     # s between sonar reads (each read also takes serial time)

# Convention used everywhere: positive angle = turn LEFT (counter-clockwise),
# same as particle theta and camera bearings. On our Arlo, wheel directions
# (left=0, right=1) turn it LEFT and (1, 0) turn it RIGHT.
# If turn(+45) ever turns RIGHT, flip this one constant (check with signcheck.py).
TURN_DIR = 1  # verified 2026-10-22 with signcheck.py: -1 turned RIGHT on turn(+45)

def turn(angle, particles):
    left  = 0
    right = 1
    if (angle * TURN_DIR < 0):
        left, right = right, left

    (arlo.go_diff(leftSpeed, rightSpeed,left, right))
    sleep((abs(angle)/360)*circleTime)
    (arlo.stop())
    sleep(eps)
    delta_theta = np.deg2rad(angle)   # CCW-positive, matches the physical turn
    for p in particles:
        particle.move_particle(p, 0, 0, delta_theta)
    # Timed turns are often 10-30% off, so heading noise must grow with the turn.
    theta_noise = max(TURN_NOISE_FRAC * abs(delta_theta), 0.5 * ROUGHEN_SIGMA_THETA)
    particle.add_uncertainty(particles, 0.0, theta_noise)

    #maybe return particle but they get modified directly
    
    

def _sonar_too_close(read, name, stop_cm=STOP_CM):
    """Read one sonar (mm -> cm). Returns a description if it is under
    stop_cm, or None if clear. Readings <= 0 are serial/sensor errors and are ignored."""
    dist = read() / 10
    if 0 < dist < stop_cm:
        return f"{name} {dist:.0f} cm"
    return None


def obstacle_ahead():
    """Front sonar at STOP_CM, then the two front-diagonal sonars at DIAG_STOP_CM."""
    return (_sonar_too_close(arlo.read_front_ping_sensor, "front")
            or _sonar_too_close(arlo.read_left_ping_sensor, "front-left", DIAG_STOP_CM)
            or _sonar_too_close(arlo.read_right_ping_sensor, "front-right", DIAG_STOP_CM))


def obstacle_behind():
    return _sonar_too_close(arlo.read_back_ping_sensor, "back")


def _straight(length, particles, forward):
    """Drive `length` cm straight (forward or backward), polling the sonar on
    the side we're moving towards and stopping early if it's too close.
    Particles are moved by the distance actually driven (estimated from
    elapsed time), which is also returned."""
    if length <= 0:
        return 0.0
    check = obstacle_ahead if forward else obstacle_behind
    word = "Driving" if forward else "Reversing"
    total_time = driveTime * length / 100
    reason = check()
    if reason is not None:
        print(f"Not {word.lower()}, obstacle {reason}")
        return 0.0

    direction = 1 if forward else 0
    start = time.monotonic()
    arlo.go_diff(leftSpeed, rightSpeed, direction, direction)
    elapsed = 0.0
    while elapsed < total_time:
        reason = check()
        elapsed = time.monotonic() - start
        if reason is not None:
            break
        sleep(SONAR_POLL_DT)
        elapsed = time.monotonic() - start
    arlo.stop()
    sleep(eps)

    driven = length * min(elapsed, total_time) / total_time
    if reason is not None:
        print(f"{word} stopped early, obstacle {reason}; moved {driven:.1f} of {length:.1f} cm")
    signed = driven if forward else -driven
    for p in particles:
        particle.move_particle(p,
                        signed * np.cos(p.getTheta()),
                        signed * np.sin(p.getTheta()),
                        0.0)
    particle.add_uncertainty(particles, SIGMA * driven, 0.0)
    return driven


def drive(length, particles):
    """Drive forward `length` cm, stopping early on the front sonar.
    Returns the distance actually driven."""
    return _straight(length, particles, forward=True)


def reverse(length, particles):
    """Back up `length` cm, stopping early on the back sonar.
    Uses the forward driveTime calibration; tune if reversing is off.
    Returns the distance actually reversed."""
    return _straight(length, particles, forward=False)


def angle_to_target(position, orientation_deg, target):
    """Smallest signed rotation (degrees) to face `target` from `position`.

    Frame: the particle-filter convention. Forward axis is +x (theta == 0 points
    along +x), orientation is measured in degrees, and a positive turn is
    counter-clockwise (heading measured from +x toward +y). This matches the
    particles' theta (whose motion is d*cos(theta), d*sin(theta)), so the caller
    passes est_pose.getTheta() (converted to degrees) directly as orientation.
    drive.turn() is also CCW-positive, so the caller passes it directly:
    drive.turn(angle).
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


def readsensor(particles):
    distance = arlo.read_front_ping_sensor()
    distanceincm = distance / 10
    print(f"FRONT SENSOR READS {distanceincm}") 
    return particles, distanceincm



if __name__ == "__main__":
    turn(360)
    drive(200)



