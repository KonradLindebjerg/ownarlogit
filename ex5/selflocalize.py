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
SIGMA_THETA = 0.04363323 # Sigma noise for when turning
SIGMA       = 0.02       # Sigma for driving distance

# Flags
showGUI  = False # Whether or not to open GUI windows
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
from lib.netplot import PlotSender

print("imported robot")
onRobot = True




# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2]
landmarks = {
    1: (-26.0, 167.0),  # Coordinates for landmark 1
    2: (55.0, 167.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks





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


def calc_likelihood(observed_distance, predicted_distance, sigma=SIGMA):
    """Return the Gaussian likelihood for a measured distance.

    The observed distance is noisy, so its residual from the predicted
    distance is compared with the sensor's Gaussian noise distribution.
    """
    residual = observed_distance - predicted_distance
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))
            * math.exp(-0.5 * (residual / sigma) ** 2))


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
    draw_world(est_pose, particles, world)

    print("Opening and initializing camera")
    if isRunningOnArlo():
        print("hej")
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(1, robottype='macbookpro', useCaptureThread=False)

    while True:
        print("In while true")

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



        
        # Use motor controls to update particles
        # XXX: Make the robot drive
        # XXX: You do this
        deltadistance = 0.2
        theta = 10
        drive.turn(theta)
        drive.drive(deltadistance)
        sleep(EPS)

        # Convert the commanded motion into filter units (cm, radians).
        # drive.drive(length) is in meters; the filter works in cm, so x100.
        CM_PER_DRIVE_UNIT = 100.0  # 1.0 drive-unit (1 m) == 100 cm
        delta_d = deltadistance * CM_PER_DRIVE_UNIT  # e.g. 0.2 m -> 20 cm
        delta_theta = np.deg2rad(theta)

        for p in particles:
            # Rotate first (robot turned, then drove)...
            particle.move_particle(p, 0.0, 0.0, delta_theta)
            # ...then translate along the particle's new heading.
            particle.move_particle(p,
                                   delta_d * np.cos(p.getTheta()),
                                   delta_d * np.sin(p.getTheta()),
                                   0.0)

        # XXX (Half C): add motion noise so the cloud can cover real drift, e.g.
        # particle.add_uncertainty(particles, sigma, sigma_theta)
        particle.add_uncertainty(particles, SIGMA, SIGMA_THETA * (delta_theta / (2 * np.pi)))

        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(objectIDs, type(None)):
            # List detected objects, keeping ONE observation per unique ID.
            # The same marker may be detected several times in a single frame;
            # objectIDs/dists/angles are numpy arrays (no .pop()), so instead of
            # mutating them we collect into a dict keyed by ID.
            observations = {}  # ID -> (dist, angle), first occurrence wins
            for i in range(len(objectIDs)):
                print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])

                # Since we know where the landmarks are, use only known IDs.
                oid = int(objectIDs[i])
                if oid not in landmarkIDs or oid in observations:
                    continue  # unknown landmark, or a duplicate of one already kept
                observations[oid] = (dists[i], angles[i])

                '''
                for p in particles:
                    predictions = {}
                    for landmark_id, (observed_distance, observed_angle) in observations.items():
                        lx, ly = landmarks[landmark_id]
                        dx = lx - p.getX()
                        dy = ly - p.getY()
                        d_pred = math.sqrt((dx**2) + (dy**2))
                        predictions[landmark_id] = d_pred
                '''
                '''    
                # Calculate distance to L1
                lx, ly = landmarks[1]
                dx = lx - p.getX()
                dy = ly - p.getY()
                d_pred = math.sqrt((dx**2) + (dy**2))
                '''


                # Calculate distance to L2
            
            for p in particles:
                likelihood = 1.0
                for landmark_id, (observed_distance, observed_angle) in observations.items():
                    landmark_x, landmark_y = landmarks[landmark_id]
                    
                    predicted_distance = math.sqrt((landmark_x - p.getX()) ** 2 + (landmark_y - p.getY()) **2)
                    likelihood *= calc_likelihood(observed_distance, predicted_distance)
                
                p.setWeight(p.getWeight() * likelihood)
            
            total_weight = sum(p.getWeight() for p in particles)
            for p in particles:
                p.setWeight(p.getWeight() / total_weight)
            
            weights = []
            for p in particles:
                weights.append(p.getWeight())
            
            cumsum = np.cumsum(weights)
            
            H20 = []
            for k in range(num_particles):
                z = np.random.rand()
                for i in range(len(cumsum)):
                    if z <= cumsum[i]:
                        H20.append(particles[i])
                        break
            particles = H20
            
            for p in particles:
                p.setWeight(1.0 / num_particles)
            
            # Compute particle weights
            # XXX: You do this finito tror jeg

            # Resampling
            # XXX: You do this finito?

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)

    
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

        if plot_sender is not None:
            plot_sender.send({
                "particles": [[p.getX(), p.getY(), p.getTheta(), p.getWeight()]
                              for p in particles],
                "est_pose": [est_pose.getX(), est_pose.getY(), est_pose.getTheta()],
                "landmarks": {str(i): list(landmarks[i]) for i in landmarkIDs},
            })
        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)
    
            # Show frame
            cv2.imshow(WIN_RF1, colour)

            # Show world
            cv2.imshow(WIN_World, world)
    
  
finally: 
    # Make sure to clean up even if an exception occurred
    
    if sendPlot and 'plot_sender' in dir() and plot_sender is not None:
        plot_sender.close()
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()

