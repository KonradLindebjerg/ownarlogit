import numpy as np
import random_numbers as rn


class Particle(object):
    """Data structure for storing particle information (state and weight)"""
    def __init__(self, x=0.0, y=0.0, theta=0.0, weight=0.0):
        self.x = x
        self.y = y
        self.theta = np.mod(theta, 2.0*np.pi)
        self.weight = weight

    def getX(self):
        return self.x
        
    def getY(self):
        return self.y
        
    def getTheta(self):
        return self.theta
        
    def getWeight(self):
        return self.weight

    def setTheta(self, val):
        self.theta = np.mod(val, 2.0*np.pi)

    def setWeight(self, val):
        self.weight = val


CORE_FRACTION = 0.9  # share of particles kept when ignoring scattered outliers


def core_particles(particles_list, keep=CORE_FRACTION):
    """Return the `keep` fraction of particles closest to the median position.

    The filter injects a few uniform random particles every update; this drops
    them (and other stragglers) so they don't skew the pose or the spread."""
    n = len(particles_list)
    if n == 0:
        return particles_list
    xs = np.array([p.getX() for p in particles_list])
    ys = np.array([p.getY() for p in particles_list])
    d2 = (xs - np.median(xs)) ** 2 + (ys - np.median(ys)) ** 2
    k = max(1, int(round(keep * n)))
    return [particles_list[i] for i in np.argsort(d2)[:k]]


def estimate_pose(particles_list):
    """Estimate the pose as the average position and orientation of the main
    particle cluster (outliers removed). Not weighted, just the sample distribution."""
    particles_list = core_particles(particles_list)
    x_sum = 0.0
    y_sum = 0.0
    cos_sum = 0.0
    sin_sum = 0.0
     
    for particle in particles_list:
        x_sum += particle.getX()
        y_sum += particle.getY()
        cos_sum += np.cos(particle.getTheta())
        sin_sum += np.sin(particle.getTheta())
        
    flen = len(particles_list)
    if flen != 0:
        x = x_sum / flen
        y = y_sum / flen
        theta = np.arctan2(sin_sum/flen, cos_sum/flen)
    else:
        x = x_sum
        y = y_sum
        theta = 0.0
    print(f"ESTIMATED POSE IS: {x}, {y} ")    
    return Particle(x, y, theta)
     
     
def move_particle(particle, delta_x, delta_y, delta_theta):
    """Move the particle by (delta_x, delta_y, delta_theta)"""
    particle.setX(particle.getX() + delta_x)
    particle.setY(particle.getY() + delta_y)
    particle.setTheta(particle.getTheta() + delta_theta)  # setTheta wraps mod 2*pi


def add_uncertainty(particles_list, sigma, sigma_theta):
    """Add some noise to each particle in the list. Sigma and sigma_theta is the noise
    variances for position and angle noise."""
    for particle in particles_list:
        particle.x += rn.randn(0.0, sigma)
        particle.y += rn.randn(0.0, sigma)
        particle.theta = np.mod(particle.theta + rn.randn(0.0, sigma_theta), 2.0 * np.pi) 


def add_uncertainty_von_mises(particles_list, sigma, theta_kappa):
    """Add some noise to each particle in the list. Sigma and theta_kappa is the noise
    variances for position and angle noise."""
    for particle in particles_list:
        particle.x += rn.randn(0.0, sigma)
        particle.y += rn.randn(0.0, sigma)
        particle.theta = np.mod(rn.rand_von_mises(particle.theta, theta_kappa), 2.0 * np.pi) - np.pi
