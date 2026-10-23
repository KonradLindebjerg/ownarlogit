import cv2
import math
import particle
import camera
import numpy as np
import time
from time import sleep
from timeit import default_timer as timer
import sys
import os

# CONSTANTS
EPS         = 0.05       # EPS for sleep time so arlo hsa time to catch up
SIGMA_THETA = 0.029320 # Sigma noise for when turning (process/motion heading noise)
SIGMA       = 2.0       # Sigma for driving distance

# of the real error and retune on the robot.
SIGMA_D         = 20.0      # cm; distance-measurement std in the likelihood
SIGMA_THETA_OBS = 0.15      # rad (~8.6 deg); bearing-measurement std in the likelihood

# Roughening added to the resampled cloud each update, to prevent particle
# impoverishment (repeated resampling collapsing to identical points).
ROUGHEN_SIGMA       = 4.0   # cm
ROUGHEN_SIGMA_THETA = 0.05  # rad (~2.9 deg)
INITIAL_POSITION_RADIUS_CM = 100.0
INITIAL_HEADING_STD_RAD = np.deg2rad(10.0)

# Each update, the least confident particles are replaced by uniform random
# ones on the board, so the filter can recover if it locks onto a wrong cluster.
RANDOM_INJECT_FRACTION = 0.05
BOARD_MARGIN_CM        = 100.0  # board = landmark bounding box grown by this

MAXDISTANCE = 0.5 # m How far th erobot can max drive in one step

# Flags
onRobot  = True  # Whether or not we are running on the Arlo robot
sendPlot = True

def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


# XXX: You need to change this path to point to where your robot.py file is located
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


from lib import drive

print("Robot drive modules loaded")
onRobot = True




# Some color constants in BGR format
CRED     = (0, 0, 255)
CGREEN   = (0, 255, 0)
CBLUE    = (255, 0, 0)
CCYAN    = (255, 255, 0)
CYELLOW  = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE   = (255, 255, 255)
CBLACK   = (0, 0, 0)


def wrapped_angle(angle):
    """Normalize an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Uniform-by-area position prior around L1 at (0, 0).
        radius = INITIAL_POSITION_RADIUS_CM * math.sqrt(np.random.rand())
        bearing = 2.0 * math.pi * np.random.rand()
        x = radius * math.cos(bearing)
        y = radius * math.sin(bearing)

        # The robot starts facing L1, with uncertainty in that initial heading.
        theta = math.atan2(-y, -x) + np.random.normal(0.0, INITIAL_HEADING_STD_RAD)
        p = particle.Particle(x, y, theta, 1.0 / num_particles)
        particles.append(p)

    return particles


def filter_confidence(particles):
    """Return (pos_std_cm, theta_std_rad) describing how tightly the cloud is clustered.

    pos_std is the spatial spread of the particles (sqrt of the summed x/y
    variances, in cm); theta_std is the circular standard deviation of the
    headings (in rad), computed from the resultant length of the unit heading
    vectors (R -> 1 as the particles align). Both shrink as the filter
    converges, so they are the natural signal for "the particles are close".
    Only the main cluster is measured, so injected random particles are ignored.
    """
    particles = particle.core_particles(particles)
    xs =np.array([p.getX() for p in particles])
    ys = np.array([p.getY() for p in particles])
    thetas = np.array([p.getTheta() for p in particles])

    pos_std = math.sqrt(xs.var() + ys.var())

    cos_mean = np.cos(thetas).mean()
    sin_mean = np.sin(thetas).mean()
    R = math.hypot(cos_mean, sin_mean)        # 1.0 = perfectly aligned
    R = min(R, 1.0 - 1e-12)                    # guard against log(0)
    theta_std = math.sqrt(-2.0 * math.log(R))

    return pos_std, theta_std


def calc_likelihood(observed_distance, predicted_distance, sigma):
    """Return the Gaussian likelihood for a measured distance.

    The observed distance is noisy, so its residual from the predicted
    distance is compared with the sensor's Gaussian noise distribution.
    """
    residual = observed_distance - predicted_distance
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))
            * math.exp(-0.5 * (residual / sigma) ** 2))


NUM_PARTICLES        = 1000
SEARCH_TURN_STEP_DEG = 60.0  # step size when spinning in place to look around
SEARCH_STEPS         = int(round(360.0 / SEARCH_TURN_STEP_DEG))  # steps in a full turn


def observe(cam, landmarkIDs, landmarks, particles):
    """Grab one frame and do one particle-filter measurement update (no motion).

    Returns (particles, objectIDs, dists, angles). The particle list may be a new
    (resampled) list, so callers must use the returned one. The raw detections
    are returned so callers can reuse this frame instead of grabbing another.
    """
    num_particles = len(particles)

    colour = cam.get_next_frame()
    objectIDs, dists, angles = cam.detect_aruco_objects(colour)
    if objectIDs is None:
        return particles, None, None, None, {} 

    # Keep ONE observation (the closest) per known landmark ID.
    observations = {}
    for i in range(len(objectIDs)):
        oid = int(objectIDs[i])
        if oid not in landmarkIDs:
            continue
        if oid in observations and dists[i] >= observations[oid][0]:
            continue
        observations[oid] = (dists[i], angles[i])

    print("OBSERVATION", observations)
    if not observations:
        return particles, objectIDs, dists, angles, observations

    for p in particles:
        likelihood = 1.0
        for landmark_id, (observed_distance, observed_angle) in observations.items():
            landmark_x, landmark_y = landmarks[landmark_id]
            dx = landmark_x - p.getX()
            dy = landmark_y - p.getY()
            predicted_distance = math.hypot(dx, dy)
            predicted_angle = wrapped_angle(math.atan2(dy, dx) - p.getTheta())
            likelihood *= calc_likelihood(observed_distance, predicted_distance, SIGMA_D)
            likelihood *= calc_likelihood(observed_angle, predicted_angle, SIGMA_THETA_OBS)
        p.setWeight(p.getWeight() * likelihood)

    total_weight = sum(p.getWeight() for p in particles)
    if total_weight <= 0.0:
        print("all likelihoods underflowed; keeping particles, reset weights")
        for p in particles:
            p.setWeight(1.0 / num_particles)
        return particles, objectIDs, dists, angles, observations

    # Resample (vectorised; the old nested loop was ~1M Python steps per frame).
    weights = np.array([p.getWeight() for p in particles]) / total_weight
    idx = np.searchsorted(np.cumsum(weights), np.random.rand(num_particles))
    idx = np.minimum(idx, num_particles - 1)
    particles = [particle.Particle(particles[i].getX(), particles[i].getY(),
                                   particles[i].getTheta(), 1.0 / num_particles)
                 for i in idx]
    # Roughen so repeated resampling cannot collapse the cloud to a few points.
    particle.add_uncertainty(particles, ROUGHEN_SIGMA, ROUGHEN_SIGMA_THETA)

    # Replace the least confident particles (lowest pre-resampling weight of
    # their parent) with uniform random ones on the board.
    num_random = int(RANDOM_INJECT_FRACTION * num_particles)
    if num_random > 0:
        xs = [c[0] for c in landmarks.values()]
        ys = [c[1] for c in landmarks.values()]
        x_min, x_max = min(xs) - BOARD_MARGIN_CM, max(xs) + BOARD_MARGIN_CM
        y_min, y_max = min(ys) - BOARD_MARGIN_CM, max(ys) + BOARD_MARGIN_CM
        for i in np.argsort(weights[idx])[:num_random]:
            particles[i] = particle.Particle(np.random.uniform(x_min, x_max),
                                             np.random.uniform(y_min, y_max),
                                             np.random.uniform(0.0, 2.0 * math.pi),
                                             1.0 / num_particles)
    return particles, objectIDs, dists, angles, observations


def localize(cam, landmarkIDs, landmarks, particles):
    """Spin in place (at least a full turn), updating the filter at every step,
    until the particle cloud is tight."""
    counter = 0
    while True:
        particles, _, _, _, observations = observe(cam, landmarkIDs, landmarks, particles)

        posconf, thetaconf = filter_confidence(particles)
        if posconf < 20 and np.rad2deg(thetaconf) < 15 and counter >= SEARCH_STEPS - 1:
            print("CONFIDENT BREAKING")
            break
        elif counter > 5 * (SEARCH_STEPS -1):
            print("NOT CONFIDENT BUT TURNED 3 TIMES SO CONTINUEING ANYWAYS")
            break



        drive.turn(SEARCH_TURN_STEP_DEG, particles)
        sleep(EPS)
        counter += 1
    return particles, observations
