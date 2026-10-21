"""Exam run: visit ArUco landmarks L1..L4 in order.

Localisation : particle filter (particle_filter.py)
Path planning: RRT (rrt.py)
Everything in ONE map frame: cm, radians CCW from +x. drive.py is the only
place that converts to the robot's clockwise-degrees turn convention.

Per landmark the loop is:
    localise -> find the target marker -> scan for obstacles ->
    plan an RRT path to a stand-off point in front of it -> drive the path,
    re-localising at each waypoint and watching the front sonar; if the sonar
    blocks, drop a virtual obstacle and replan.

Run on the Pi:  python main.py
Live plot (laptop, started first):  PLOT_PORT=5005 python plot_client.py
and set this machine's view of the laptop via the PLOT_HOST env var.
"""

import math
import os
import sys
from time import sleep

import drive
import geometry
import particle_filter as pf
import landmarkmapping as lm
import rrt as rrtmod
import netplot

# --- Flags ----------------------------------------------------------------
DEBUG     = True
PLOT_HOST = os.environ.get("PLOT_HOST")            # None -> run headless
PLOT_PORT = int(os.environ.get("PLOT_PORT", 5005))

# --- World ----------------------------------------------------------------
ROBOT_RADIUS = 22.5   # cm
LANDMARK_R   = lm.landmarkR  # 15 cm, matches detectLandmark / obstacle inflation
STANDOFF_CM  = 50.0   # stop this far in front of a target landmark

# ArUco id -> map coordinate (cm)
LANDMARKS = {
    1:    (0.0,   0.0),
    7:    (0.0,   300.0),
    100:  (400.0, 0.0),
    1000: (400.0, 300.0),
}
VISIT_ORDER = [1, 7, 100, 1000]

play_area = [-120 + ROBOT_RADIUS, 620 - ROBOT_RADIUS,
             -120 + ROBOT_RADIUS, 520 - ROBOT_RADIUS]
rand_area = [min(play_area[0], play_area[2]), max(play_area[1], play_area[3])]

# --- Tuning ---------------------------------------------------------------
NUM_PARTICLES   = 1000
STARTUP_STEPS   = 16          # rotation steps while first localising
STARTUP_STEP    = 25.0        # deg per startup rotation (hardware CW+)
FIND_STEPS      = 18          # rotation steps while hunting a target marker
FIND_STEP       = 20.0        # deg per hunt rotation
SCAN_OFFSETS    = [0.0, 30.0, -60.0, 30.0]  # relative CW-deg sweep, nets to 0
REFINE_FRAMES   = 3           # measurement updates taken while standing still
DRIVE_CHUNK_CM  = 20.0        # forward step between sonar checks
SONAR_STOP_MM   = 300         # treat < this as blocked
MAX_REPLANS     = 4           # per leg


def log(*a):
    if DEBUG:
        print(*a)


# --------------------------------------------------------------------------
# Localisation helpers
# --------------------------------------------------------------------------
def refine(cam, particles, frames=REFINE_FRAMES):
    """Take a few stationary measurement updates to sharpen the cloud."""
    for _ in range(frames):
        particles = pf.update(particles, pf.observe(cam), LANDMARKS)
        sleep(drive.eps)
    return particles


def startup_localize(cam, particles):
    """Rotate until at least two distinct landmarks have been seen, updating
    the filter on every frame, then refine in place."""
    seen = set()
    for _ in range(STARTUP_STEPS):
        obs = pf.observe(cam)
        seen.update(oid for oid in obs if oid in LANDMARKS)
        particles = pf.update(particles, obs, LANDMARKS)
        if len(seen) >= 2:
            break
        drive.turn(STARTUP_STEP, particles)
        sleep(drive.eps)
    particles = refine(cam, particles)
    est = pf.estimate(particles)
    log("STARTUP pose: x=%.0f y=%.0f th=%.0f deg  (saw %s)"
        % (est.getX(), est.getY(), math.degrees(est.getTheta()), sorted(seen)))
    return particles, est


def find_target(cam, particles, target_id):
    """Rotate in place until `target_id` is in view, updating the filter."""
    for _ in range(FIND_STEPS):
        obs = pf.observe(cam)
        particles = pf.update(particles, obs, LANDMARKS)
        if target_id in obs:
            log("Found target", target_id)
            return True, particles
        drive.turn(FIND_STEP, particles)
        sleep(drive.eps)
    log("WARNING: target", target_id, "not found after full sweep")
    return False, particles


# --------------------------------------------------------------------------
# Obstacle scanning
# --------------------------------------------------------------------------
def scan_obstacles(cam, particles, target_id):
    """Sweep a few headings, collect detected markers in the map frame.

    Returns (obstacle_list, particles). The target landmark is excluded (it is
    the goal). Known landmarks are added from their surveyed coordinates rather
    than their noisy detections; only genuinely unknown markers use detection.
    """
    detected = {}
    for k, offset in enumerate(SCAN_OFFSETS):
        if offset != 0.0:
            drive.turn(offset, particles)
            sleep(drive.eps)
        # Update the filter from this pose, then map obstacle geometry.
        particles = pf.update(particles, pf.observe(cam), LANDMARKS)
        est = pf.estimate(particles)
        mapped = geometry.camera_obstacles_to_map(
            lm.detectLandmark(cam), est.getX(), est.getY(), est.getTheta())
        for (mx, my, r, oid) in mapped:
            if oid in LANDMARKS:
                continue  # use surveyed coordinates for known landmarks
            detected[oid] = (mx, my, r, oid)

    obstacles = list(detected.values())
    # Every known landmark except the one we are driving to is an obstacle.
    for oid, (lx, ly) in LANDMARKS.items():
        if oid != target_id:
            obstacles.append((lx, ly, LANDMARK_R, oid))
    log("Scan obstacles:", obstacles)
    return obstacles, particles


def front_blocked():
    val = drive.arlo.read_front_ping_sensor()
    return 0 < val < SONAR_STOP_MM


# --------------------------------------------------------------------------
# Driving a planned path
# --------------------------------------------------------------------------
def drive_path(cam, particles, path):
    """Drive the RRT path (goal-first) waypoint by waypoint.

    Returns (reached, particles, block_point). `block_point` is a map (x, y)
    where the sonar stopped us, or None.
    """
    waypoints = list(reversed(path))[1:]  # start->goal, drop the start point
    for wp in waypoints:
        particles = pf.update(particles, pf.observe(cam), LANDMARKS)  # re-localise
        est = pf.estimate(particles)
        dtheta = geometry.angle_to_target(est.getX(), est.getY(),
                                          est.getTheta(), wp)
        drive.turn_to(dtheta, particles)

        dist = geometry.distance([est.getX(), est.getY()], wp)
        moved = 0.0
        while moved < dist - 1e-3:
            if front_blocked():
                est = pf.estimate(particles)
                d_cm = SONAR_STOP_MM / 10.0
                block = (est.getX() + d_cm * math.cos(est.getTheta()),
                         est.getY() + d_cm * math.sin(est.getTheta()))
                log("Sonar block at", block)
                return False, particles, block
            step = min(DRIVE_CHUNK_CM, dist - moved)
            drive.drive(step, particles)
            moved += step
    return True, particles, None


def go_to_landmark(cam, particles, target_id, plot_sender):
    """Plan + drive to a stand-off point in front of `target_id`, replanning
    around sonar blocks."""
    found, particles = find_target(cam, particles, target_id)
    extra_blocks = []  # virtual obstacles discovered by sonar on this leg

    for attempt in range(MAX_REPLANS):
        obstacles, particles = scan_obstacles(cam, particles, target_id)
        obstacles = obstacles + extra_blocks

        est = pf.estimate(particles)
        start = [est.getX(), est.getY()]
        goal = geometry.standoff_goal(start, LANDMARKS[target_id], STANDOFF_CM)

        planner = rrtmod.RRT(
            start=start, goal=goal, obstacle_list=obstacles,
            rand_area=rand_area, play_area=play_area,
            robot_radius=ROBOT_RADIUS, robot_orientation=est.getTheta(),
            path_resolution=5.0, plot_sender=plot_sender)
        path = planner.planning(animation=plot_sender is not None)

        if path is None:
            log("No path to", target_id, "(attempt", attempt, ") - re-scanning")
            drive.turn(FIND_STEP, particles)  # nudge viewpoint and retry
            continue

        log("Planned %d-point path to %s" % (len(path), target_id))
        reached, particles, block = drive_path(cam, particles, path)
        if reached:
            log("Reached standoff for", target_id)
            return True, particles
        if block is not None:
            extra_blocks.append((block[0], block[1], ROBOT_RADIUS, -1))

    log("Giving up on", target_id, "after", MAX_REPLANS, "attempts")
    return False, particles


# --------------------------------------------------------------------------
def main():
    plot_sender = None
    if PLOT_HOST:
        plot_sender = netplot.PlotSender(PLOT_HOST, PLOT_PORT)

    cam = __import__("camera").Camera(0, robottype='arlo', useCaptureThread=False)
    sleep(drive.eps)

    try:
        particles = pf.initialize_particles(NUM_PARTICLES)
        particles, _ = startup_localize(cam, particles)

        for target_id in VISIT_ORDER:
            log("\n=== Heading to landmark id", target_id, "===")
            particles = refine(cam, particles)
            ok, particles = go_to_landmark(cam, particles, target_id, plot_sender)
            if not ok:
                log("Aborting route: could not reach", target_id)
                break
        else:
            log("\nRoute complete: visited", VISIT_ORDER)
    finally:
        drive.arlo.stop()
        cam.terminateCaptureThread()
        if plot_sender is not None:
            plot_sender.close()


if __name__ == "__main__":
    main()
