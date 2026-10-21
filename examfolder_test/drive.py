"""Timing-based motion for the Arlo, plus the matching particle motion model.

ANGLE CONVENTION (read this once, it is the whole ballgame)
----------------------------------------------------------
Everything *above* this module - the particle filter, the RRT planner, the
obstacle geometry - works in the MAP frame:

    * x, y in centimetres,
    * theta in radians, measured CCW from the +x axis (theta == 0 points along
      +x, forward motion is (d*cos theta, d*sin theta)),
    * a positive turn is counter-clockwise (left).

The Arlo hardware turn is the OPPOSITE sign: go_diff with (left=0, right=1)
spins the robot clockwise, so `turn()` below takes a CLOCKWISE-positive angle in
DEGREES. Callers that think in the map frame therefore negate and convert:

    drive.turn(-math.degrees(delta_theta_ccw), particles)

`turn()` moves the particle cloud by the correct CCW map amount internally, so
callers never touch particle theta themselves. This is the single place the two
conventions meet.

These are calibration values - retune `leftSpeed/rightSpeed/circleTime/
driveTime` per robot.
"""

from time import sleep
import math

import numpy as np

import robot
import particle

# One shared Robot instance for the whole program (the class is not
# thread-safe and opens /dev/ttyACM0 on construction, so there must be exactly
# one). main.py reads sonar through drive.arlo.
arlo = robot.Robot()

leftSpeed   = 68
rightSpeed  = 66
circleTime  = 2.75      # seconds for a full 360 deg spin at the tuned powers
driveTime   = 2.25      # seconds to drive 100 cm at the tuned powers
eps         = 0.3       # settle time after each motor command

# Motion-model noise added to the cloud after each move.
SIGMA               = 0.02    # cm of position noise per cm driven
SIGMA_THETA         = 0.029320
ROUGHEN_SIGMA_THETA = 0.05    # rad


def turn(angle_deg, particles):
    """Turn in place by `angle_deg` CLOCKWISE-positive degrees (hardware sign).

    The particle cloud is rotated by the matching CCW map amount
    (-angle_deg) so the filter stays consistent with the robot.
    """
    left, right = 0, 1
    if angle_deg < 0:
        left, right = right, left

    arlo.go_diff(leftSpeed, rightSpeed, left, right)
    sleep((abs(angle_deg) / 360.0) * circleTime)
    arlo.stop()
    sleep(eps)

    # Map frame is CCW-positive, hardware angle is CW-positive -> negate.
    delta_theta = np.deg2rad(-angle_deg)
    for p in particles:
        particle.move_particle(p, 0.0, 0.0, delta_theta)
    theta_noise = max(SIGMA_THETA * abs(delta_theta) / (2.0 * np.pi),
                      0.5 * ROUGHEN_SIGMA_THETA)
    particle.add_uncertainty(particles, 0.0, theta_noise)


def drive(length_cm, particles):
    """Drive straight forward `length_cm` centimetres and move the cloud."""
    if length_cm <= 0.0:
        return
    arlo.go_diff(leftSpeed, rightSpeed, 1, 1)
    sleep(driveTime * length_cm / 100.0)
    arlo.stop()
    sleep(eps)

    for p in particles:
        particle.move_particle(p,
                               length_cm * np.cos(p.getTheta()),
                               length_cm * np.sin(p.getTheta()),
                               0.0)
    particle.add_uncertainty(particles, SIGMA * length_cm, 0.0)


def turn_to(delta_theta_rad, particles):
    """Convenience: turn by a CCW-positive MAP-frame angle given in radians."""
    turn(-math.degrees(delta_theta_rad), particles)
