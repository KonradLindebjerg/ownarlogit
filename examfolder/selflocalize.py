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

# --- Observation-model (likelihood) noise ---------------------------------
# These are the spread of the measurement likelihood and must reflect the TOTAL
# sensor+model uncertainty across the operating range (focal-length error,
# range-dependent bias, marker size, mounting), NOT the fixed-pose repeatability
# that calibratesigma.py reports. The calibrated values (SIGMA_D=0.38 cm,
# bearing=1.7 deg) are ~50x too tight: with 1000 particles spread over a
# ~600x600 cm area (~19 cm spacing), almost none land inside such a narrow
# likelihood shell, so a single frame collapses the whole cloud onto 1-2
# samples -> overconfident on one landmark, then stuck. Keep these on the scale
# of the real error and retune on the robot.
SIGMA_D         = 20.0      # cm; distance-measurement std in the likelihood
SIGMA_THETA_OBS = 0.15      # rad (~8.6 deg); bearing-measurement std in the likelihood

# Roughening added to the resampled cloud each update, to prevent particle
# impoverishment (repeated resampling collapsing to identical points).
ROUGHEN_SIGMA       = 4.0   # cm
ROUGHEN_SIGMA_THETA = 0.05  # rad (~2.9 deg)

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
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles


def calc_likelihood(observed_distance, predicted_distance, sigma):
    """Return the Gaussian likelihood for a measured distance.

    The observed distance is noisy, so its residual from the predicted
    distance is compared with the sensor's Gaussian noise distribution.
    """
    residual = observed_distance - predicted_distance
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))
            * math.exp(-0.5 * (residual / sigma) ** 2))


# Main program #
# TODO Change the function name to not be main
def localize(cam, landmarkIDs, landmarks, particles):
    num_particles = 1000
    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    
    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)
    
    
    #last_goal = MIDDLEPOINT
    #GOAL_DISTANCE_TOLERANCE_CM = 40.0
    #GOAL_ANGLE_TOLERANCE_RAD = np.deg2rad(10.0)
    SEARCH_TURN_STEP_DEG = 20.0
    search_mode = False
    frame_count = 0
    seen_landmark_ids = set()
    landmarks_acquired = False
    counter = 0
    
    while True:
        frame_count += 1
    
        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
            elif action == ord('x'): # Backwards
                velocity -= 4.0
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
            elif action == ord('d'): # Right
                angular_velocity -= 0.2
    
        # Fetch next frame
        colour = cam.get_next_frame()
    
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        observations = {}
        weight_status = "no valid landmark observations"
        if not isinstance(objectIDs, type(None)):
            # List detected objects, keeping ONE observation per unique ID.
            for i in range(len(objectIDs)):
                oid = int(objectIDs[i])
                if oid not in landmarkIDs:
                    detection_status = "ignored: unknown landmark ID"
                elif oid in observations and dists[i] >= observations[oid][0]:
                    detection_status = "ignored: farther duplicate"
                else:
                    detection_status = "used: closest detection"
                    observations[oid] = (dists[i], angles[i])
    
                if detection_status != "used":
                    continue
                    
            seen_landmark_ids.update(observations)
            print(seen_landmark_ids)
            landmarks_acquired = all(landmark_id in seen_landmark_ids
                                     for landmark_id in landmarkIDs)
    
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
            if not observations:
                weight_status = "prior only; no recognized landmarks"
            elif total_weight > 0.0:
                for p in particles:
                    p.setWeight(p.getWeight() / total_weight)
    
                effective_sample_size = 1.0 / sum(p.getWeight() ** 2 for p in particles)
                max_weight = max(p.getWeight() for p in particles)
                weight_status = (f"updated; ESS={effective_sample_size:.0f}/{num_particles}, "
                                 f"max weight={max_weight:.4f}")
    
                weights = [p.getWeight() for p in particles]
                cumsum = np.cumsum(weights)
                resampled = []
                for _ in range(num_particles):
                    z = np.random.rand()
                    for i in range(len(cumsum)):
                        if z <= cumsum[i]:
                            src = particles[i]
                            resampled.append(particle.Particle(
                                src.getX(), src.getY(), src.getTheta(),
                                1.0 / num_particles,
                            ))
                            break
                particles = resampled
                # Roughen the resampled cloud so repeated resampling cannot
                # collapse it to a few identical points (particle
                # impoverishment -> locking onto one landmark and getting stuck).
                particle.add_uncertainty(particles, ROUGHEN_SIGMA, ROUGHEN_SIGMA_THETA)

                # POSSIBLE TODO: ADD RANDOM UNIFORM PARTICLES IN THE MIX HERE

            else:
                for p in particles:
                    p.setWeight(1.0 / num_particles)
                weight_status = "all likelihoods underflowed; reset weights uniformly"
            search_mode = not landmarks_acquired
    
            cam.draw_aruco_objects(colour)
        else:
            # Search until both landmark IDs have been observed at least once.
            search_mode = not landmarks_acquired
            for p in particles:
                p.setWeight(1.0 / num_particles)
            weight_status = "no markers detected; weights unchanged"
    
        est_pose = particle.estimate_pose(particles)
    

        
        # IF EST_POSE IS WITHIN STASFIABLE CONDITION RETURN EST_POSE TO main.py
        if counter > 18:
            break


        # If the robot is near a single visible landmark, treat it as a tracking
        # target temporarily; otherwise keep moving toward the midpoint.
        # If no valid landmark was seen on the current frame, force a search
        # manoeuvre instead of continuing toward a stale goal.
        should_search = search_mode
    
        turn_cmd_deg = 0.0
        # We probably shouldt be driving in the exam solution
        drive_cmd_cm = 0.0
    
        if len(seen_landmark_ids) < 2:
            turn_cmd_deg = SEARCH_TURN_STEP_DEG
            drive.turn(turn_cmd_deg, particles)
            sleep(EPS)
        else:
            est_pose = particle.estimate_pose(particles)
            
        # The below was used for for driing towards the middlepoint in ex5, maybe delete
        #else:
        #    turn_cmd_deg = -goal_angle
        #    drive.turn(turn_cmd_deg)
        #    sleep(EPS)
        #    MAX_DRIVE_CM = 50.0
        #    drive_cmd_cm = min(goal_distance, MAX_DRIVE_CM)
        #    drive.drive(drive_cmd_cm)
        #    sleep(EPS)
    
        # a floor so theta diversity does not decay to zero when driving straight.
        theta_noise = max(SIGMA_THETA * abs(delta_theta) / (2 * np.pi), 0.5 * ROUGHEN_SIGMA_THETA)
        particle.add_uncertainty(particles, SIGMA, theta_noise)
        counter += 1
    return est_pose, particles
