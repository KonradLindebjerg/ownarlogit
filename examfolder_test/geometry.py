"""Pure geometry in the single MAP frame (+x forward, CCW+, radians, cm).

No hardware, no state - just the coordinate math that the planner and the
drive loop share, kept in one place so the convention cannot drift.
"""

import math


def wrap(angle):
    """Normalise a radian angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_to_target(x, y, theta, target):
    """Smallest signed turn (radians, CCW+) to face `target` from pose (x,y,theta).

    Same convention as the particle filter: heading measured CCW from +x. A
    positive result means turn left. Hand it to drive.turn via
    drive.turn_to(result, particles) (which applies the hardware sign flip).
    """
    desired = math.atan2(target[1] - y, target[0] - x)
    return wrap(desired - theta)


def distance(a, b):
    """Euclidean distance between two [x, y] points (cm)."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def camera_obstacles_to_map(obstacles, x, y, theta):
    """Rotate+translate camera-frame detections into the map frame.

    `obstacles` is what landmarkmapping.detectLandmark returns: a list of
    (xr, z, radius, id) with xr = metres? no - centimetres to the RIGHT and
    z = centimetres FORWARD (camera frame). (x, y, theta) is the robot pose in
    the map frame when the frame was captured.

    Robot forward f = (cos th, sin th); robot right r = (sin th, -cos th)
    (forward rotated -90 deg). A detection at camera (xr right, z forward) is
    therefore at map point  (x, y) + z*f + xr*r.
    """
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    mapped = []
    for (xr, z, radius, oid) in obstacles:
        mx = x + z * cos_t + xr * sin_t
        my = y + z * sin_t - xr * cos_t
        mapped.append((mx, my, radius, oid))
    return mapped


def standoff_goal(start, landmark, standoff_cm):
    """A point `standoff_cm` short of `landmark`, on the line from it to `start`.

    Driving to the landmark centre would mean driving into it; we stop a robot
    radius + marker radius + margin in front instead.
    """
    dx, dy = start[0] - landmark[0], start[1] - landmark[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return [landmark[0], landmark[1]]
    return [landmark[0] + dx / d * standoff_cm,
            landmark[1] + dy / d * standoff_cm]
