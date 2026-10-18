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
SIGMA_THETA = 0.029320 # Sigma noise for when turning
SIGMA       = 2.0       # Sigma for driving distance
# Measurement noise for the ArUco distance reading, in cm. This is the spread
# of the sensor likelihood and must be on the scale of the distances (cm),
# NOT the tiny motion sigma above. Too small -> all weights underflow to 0.
SIGMA_D     = 0.379435       # cm; calibrated distance-measurement standard deviation
MAXDISTANCE = 0.5 # m How far th erobot can max drive in one step

# Flags
showGUI  = False # Whether or not to open GUI windows
onRobot  = True  # Whether or not we are running on the Arlo robot
sendPlot = True
DEBUG    = True


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


# XXX: You need to change this path to point to where your robot.py file is located
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


from lib import drive
from lib.netplot import PlotSender

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

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 3]
landmarks = {
    1: (0.0, 0.0),    # Coordinates for landmark 1
    3: (300.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks

l1x, l1y = landmarks[1]
l2x, l2y = landmarks[3]

MIDDLEPOINT = ((l1x + l2x) / 2.0, (l1y + l2y) / 2.0)


def wrapped_angle(angle):
    """Normalize an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi




def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 0
    offsetY = 0

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
         ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)



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


def estimate_weighted_pose(particles):
    total_weight = sum(p.getWeight() for p in particles)
    if total_weight <= 0.0:
        return particle.estimate_pose(particles)

    x = sum(p.getWeight() * p.getX() for p in particles) / total_weight
    y = sum(p.getWeight() * p.getY() for p in particles) / total_weight
    cos_theta = sum(p.getWeight() * math.cos(p.getTheta()) for p in particles)
    sin_theta = sum(p.getWeight() * math.sin(p.getTheta()) for p in particles)
    theta = math.atan2(sin_theta, cos_theta)
    return particle.Particle(x, y, theta)


# Main program #
try:
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)


    # Initialize particles
    num_particles = 1000
    particles = initialize_particles(num_particles)

    PLOT_HOST = os.environ.get("PLOT_HOST", "172.20.10.8")   # laptop IP
    PLOT_PORT = int(os.environ.get("PLOT_PORT", "5005"))
    plot_sender = PlotSender(PLOT_HOST, PLOT_PORT) if sendPlot else None

    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    
    
    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec

    # Initialize the robot (XXX: You do this)
    # Robot gets init in lib/drive.python
    
    

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    #draw_world(est_pose, particles, world)

    print("Initializing camera...")
    if isRunningOnArlo():
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(1, robottype='macbookpro', useCaptureThread=False)

    last_goal = MIDDLEPOINT
    GOAL_DISTANCE_TOLERANCE_CM = 10.0
    SEARCH_TURN_STEP_DEG = 20.0
    search_mode = False
    frame_count = 0
    seen_landmark_ids = set()
    landmarks_acquired = False

    if DEBUG:
        print(f"Localization started: {num_particles} particles, landmarks={landmarkIDs}, "
              f"distance sigma={SIGMA_D:.2f} cm, angle sigma={np.rad2deg(SIGMA_THETA):.1f} deg")

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

                if DEBUG:
                    print(f"  Detection ID {oid}: {float(dists[i]):.1f} cm, "
                          f"{math.degrees(float(angles[i])):+.1f} deg ({detection_status})")

                if detection_status != "used":
                    continue

            previous_weights = [p.getWeight() for p in particles]
            for p in particles:
                likelihood = 1.0
                for landmark_id, (observed_distance, observed_angle) in observations.items():
                    landmark_x, landmark_y = landmarks[landmark_id]

                    dx = landmark_x - p.getX()
                    dy = landmark_y - p.getY()
                    predicted_distance = math.hypot(dx, dy)
                    predicted_angle = wrapped_angle(math.atan2(dy, dx) - p.getTheta())

                    likelihood *= calc_likelihood(observed_distance, predicted_distance, SIGMA_D)
                    likelihood *= calc_likelihood(observed_angle, predicted_angle, SIGMA_THETA)

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

                seen_landmark_ids.update(observations)
                landmarks_acquired = all(landmark_id in seen_landmark_ids
                                         for landmark_id in landmarkIDs)

                if landmarks_acquired and effective_sample_size < num_particles / 2.0:
                    weights = [p.getWeight() for p in particles]
                    cumsum = np.cumsum(weights)
                    resampled = []
                    for _ in range(num_particles):
                        z = np.random.rand()
                        index = np.searchsorted(cumsum, z, side="left")
                        index = min(index, len(particles) - 1)
                        src = particles[index]
                        resampled.append(particle.Particle(
                            src.getX(), src.getY(), src.getTheta(),
                            1.0 / num_particles,
                        ))
                    particles = resampled
            else:
                if landmarks_acquired:
                    for p, previous_weight in zip(particles, previous_weights):
                        p.setWeight(previous_weight)
                    weight_status = "measurement rejected: likelihood underflow"
                else:
                    for p in particles:
                        p.setWeight(1.0 / num_particles)
                    weight_status = "measurement underflow; kept particles, reset weights"

            if len(observations) >= 2:
                # Both landmarks are visible: target the midpoint between them.
                observed_ids = sorted(observations.keys())
                l1x, l1y = landmarks[observed_ids[0]]
                l2x, l2y = landmarks[observed_ids[1]]
                last_goal = ((l1x + l2x) / 2.0, (l1y + l2y) / 2.0)
            search_mode = not landmarks_acquired

            cam.draw_aruco_objects(colour)
        else:
            # Search until both landmark IDs have been observed at least once.
            search_mode = not landmarks_acquired
            weight_status = "no markers detected; weights unchanged"

        est_pose = estimate_weighted_pose(particles)

        # Use the current estimate and the latest observed target to decide whether
        # to stop or move. Doing this after the camera update prevents stale
        # estimates from driving the robot past the correct midpoint.
        target = last_goal
        goal_angle = drive.angle_to_target((est_pose.getX(), est_pose.getY()), np.rad2deg(est_pose.getTheta()), target)
        goal_distance = math.hypot(est_pose.getX() - target[0], est_pose.getY() - target[1])
        heading_error = abs(wrapped_angle(math.atan2(target[1] - est_pose.getY(), target[0] - est_pose.getX()) - est_pose.getTheta()))

        if DEBUG:
            mode = "SEARCH: acquire landmarks" if search_mode else "NAVIGATE: midpoint"
            spread_x = float(np.std([p.getX() for p in particles]))
            spread_y = float(np.std([p.getY() for p in particles]))
            observed_ids = sorted(observations)
            print(f"Frame {frame_count}: detections={observed_ids or 'none'}, "
                  f"seen={sorted(seen_landmark_ids)}; {weight_status}")
            print(f"  Pose estimate: x={est_pose.getX():.1f} cm, y={est_pose.getY():.1f} cm, "
                  f"heading={math.degrees(est_pose.getTheta()):+.1f} deg "
                  f"(particle spread x={spread_x:.1f}, y={spread_y:.1f} cm)")
            print(f"  Target: x={target[0]:.1f} cm, y={target[1]:.1f} cm; "
                  f"distance={goal_distance:.1f} cm, bearing error={goal_angle:+.1f} deg, "
                  f"heading error={math.degrees(heading_error):.1f} deg; mode={mode}")

        if not search_mode and goal_distance <= GOAL_DISTANCE_TOLERANCE_CM:
            print(f"Stopping: midpoint is within {GOAL_DISTANCE_TOLERANCE_CM:.1f} cm "
                  f"(estimated distance {goal_distance:.1f} cm).")
            drive.arlo.stop()
            sleep(EPS)
            break

        # If the robot is near a single visible landmark, treat it as a tracking
        # target temporarily; otherwise keep moving toward the midpoint.
        # If no valid landmark was seen on the current frame, force a search
        # manoeuvre instead of continuing toward a stale goal.
        should_search = search_mode

        turn_cmd_deg = 0.0
        drive_cmd_cm = 0.0

        if should_search:
            turn_cmd_deg = SEARCH_TURN_STEP_DEG
            drive.turn(turn_cmd_deg)
            sleep(EPS)
        else:
            turn_cmd_deg = -goal_angle
            drive.turn(turn_cmd_deg)
            sleep(EPS)
            MAX_DRIVE_CM = 50.0
            drive_cmd_cm = min(goal_distance, MAX_DRIVE_CM)
            drive.drive(drive_cmd_cm / 100.0)
            sleep(EPS)

        if DEBUG:
            print(f"  Command: motor turn {turn_cmd_deg:+.1f} deg, drive {drive_cmd_cm:.1f} cm")
            print(f"  Particle heading update: {-turn_cmd_deg:+.1f} deg")

        # Move the particles by exactly what we commanded the robot to do.
        # Motor turn and particle heading use opposite sign conventions.
        delta_theta = np.deg2rad(-turn_cmd_deg)
        delta_d = drive_cmd_cm

        for p in particles:
            particle.move_particle(p, 0.0, 0.0, delta_theta)
            particle.move_particle(p,
                                   delta_d * np.cos(p.getTheta()),
                                   delta_d * np.sin(p.getTheta()),
                                   0.0)

        particle.add_uncertainty(particles, SIGMA, SIGMA_THETA * (delta_theta / (2 * np.pi)))

        if plot_sender is not None:
            plot_sender.send({
                "particles": [[p.getX(), p.getY(), p.getTheta(), p.getWeight()]
                              for p in particles],
                "est_pose": [est_pose.getX(), est_pose.getY(), est_pose.getTheta()],
                "landmarks": {str(i): list(landmarks[i]) for i in landmarkIDs},
            })
        if showGUI:
            draw_world(est_pose, particles, world)
            cv2.imshow(WIN_RF1, colour)
            cv2.imshow(WIN_World, world)

        # Drive towards middle of landmarks

    cam.terminateCaptureThread()
    
  
finally: 
    # Make sure to clean up even if an exception occurred
    
    if sendPlot and 'plot_sender' in dir() and plot_sender is not None:
        plot_sender.close()
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    #cam.terminateCaptureThread()

