"""Particle-filter self-localisation (MAP frame: cm, radians CCW from +x).

Responsibilities split cleanly:

    * MOTION (predict) is applied by drive.turn / drive.drive, which already
      move the cloud and add process noise. This module never moves particles
      for motion.
    * MEASUREMENT (update) is `update()`: reweight by the Gaussian likelihood
      of each observed ArUco (distance + bearing), normalise, low-variance
      resample, then roughen.

The observation model's SIGMA_D / SIGMA_THETA_OBS are deliberately much larger
than the fixed-pose repeatability calibratesigma.py reports: they must cover
the TOTAL error over the operating range, or the cloud collapses onto one or
two particles after a single frame. Retune on the robot.
"""

import math

import numpy as np

import particle

# Observation-model (likelihood) spread.
SIGMA_D         = 20.0   # cm, distance measurement std
SIGMA_THETA_OBS = 0.15   # rad (~8.6 deg), bearing measurement std

# Roughening added after each resample to fight particle impoverishment.
ROUGHEN_SIGMA       = 4.0   # cm
ROUGHEN_SIGMA_THETA = 0.05  # rad

# Sampling box for the initial cloud (cm), a little larger than the play area.
INIT_X = (-150.0, 500.0)
INIT_Y = (-150.0, 450.0)


def initialize_particles(num_particles):
    particles = []
    w = 1.0 / num_particles
    for _ in range(num_particles):
        p = particle.Particle(
            np.random.uniform(*INIT_X),
            np.random.uniform(*INIT_Y),
            np.random.uniform(0.0, 2.0 * np.pi),
            w)
        particles.append(p)
    return particles


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _gauss(residual, sigma):
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))
            * math.exp(-0.5 * (residual / sigma) ** 2))


def observe(cam):
    """Capture one frame and return {id: (distance_cm, bearing_rad)}.

    Keeps the closest detection per id (a marker seen from the side can be
    detected twice). Bearing is CCW-positive (camera is positive-to-the-left),
    matching the map frame.
    """
    frame = cam.get_next_frame()
    ids, dists, angles = cam.detect_aruco_objects(frame)
    observations = {}
    if ids is None:
        return observations
    for i in range(len(ids)):
        oid = int(ids[i])
        if oid not in observations or dists[i] < observations[oid][0]:
            observations[oid] = (float(dists[i]), float(angles[i]))
    return observations


def update(particles, observations, landmark_coords):
    """Measurement update for one frame. Returns the (possibly new) cloud.

    `observations` : {id: (distance_cm, bearing_rad)} from observe().
    `landmark_coords` : {id: (x, y)} known map positions.
    Only observations whose id is a known landmark are used.
    """
    used = {oid: obs for oid, obs in observations.items()
            if oid in landmark_coords}
    if not used:
        return particles  # nothing to learn from this frame

    for p in particles:
        likelihood = 1.0
        px, py, pth = p.getX(), p.getY(), p.getTheta()
        for oid, (obs_d, obs_a) in used.items():
            lx, ly = landmark_coords[oid]
            dx, dy = lx - px, ly - py
            pred_d = math.hypot(dx, dy)
            pred_a = _wrap(math.atan2(dy, dx) - pth)
            likelihood *= _gauss(obs_d - pred_d, SIGMA_D)
            likelihood *= _gauss(_wrap(obs_a - pred_a), SIGMA_THETA_OBS)
        p.setWeight(p.getWeight() * likelihood)

    total = sum(p.getWeight() for p in particles)
    n = len(particles)
    if total <= 0.0:
        # Everything underflowed - reset to uniform rather than divide by zero.
        for p in particles:
            p.setWeight(1.0 / n)
        return particles

    for p in particles:
        p.setWeight(p.getWeight() / total)

    particles = _low_variance_resample(particles)
    particle.add_uncertainty(particles, ROUGHEN_SIGMA, ROUGHEN_SIGMA_THETA)
    return particles


def _low_variance_resample(particles):
    n = len(particles)
    weights = np.array([p.getWeight() for p in particles])
    positions = (np.arange(n) + np.random.uniform()) / n
    cumsum = np.cumsum(weights)
    cumsum[-1] = 1.0
    resampled, i = [], 0
    for pos in positions:
        while pos > cumsum[i]:
            i += 1
        src = particles[i]
        resampled.append(
            particle.Particle(src.getX(), src.getY(), src.getTheta(), 1.0 / n))
    return resampled


def effective_sample_size(particles):
    s = sum(p.getWeight() ** 2 for p in particles)
    return (1.0 / s) if s > 0 else 0.0


def estimate(particles):
    return particle.estimate_pose(particles)
