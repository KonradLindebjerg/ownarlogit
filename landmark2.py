# ======= TODO list =======
#
# Vi skal kalibere kameraet så vi får Camera matrix (som jeg tror skal bruge focal length som vi udregnede før) og distortion matrix som jeg ikke ved noget om


from time import sleep

import robot
import numpy as np
import picamera2
import time
import os
import cv2 # Import the OpenCV library

# Create a robot object and initialize
arlo = robot.Robot()

# Open a camera device for capturing
cam = cv2.VideoCapture(0)

leftSpeed = 68  
rightSpeed = 65
imageSize = (1640, 1232)
FPS = 30

atlandmark = False


cam = picamera2.Picamera2()

frame_duration_limit = int(1/FPS * 1000000)

# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration(
    {"size": imageSize, "format": "RGB888"},
    controls={
        "FrameDurationLimits": (
            frame_duration_limit,
            frame_duration_limit
        ),
        "ScalerCrop": (0, 0, 3280, 2464)
    },
    queue=False
)

cam.configure(picam2_config)
cam.start(show_preview=False)

print(cam.camera_configuration())

time.sleep(1)


# Folder for images
folder = "camera_images"
os.makedirs(folder, exist_ok=True)

print("Saving images to:", folder)

image_number = 0

# Moving towards the found lander 

def travelLandmark(tvec):
    x = tvec[0]
    distance = tvec[2]
    tolerance = 0.05 * distance

    if x > tolerance:
        print("Moving right!")
        arlo.go_diff(leftSpeed*0.5, rightSpeed*0.5, 1, 0)
        sleep(0.2)
        arlo.stop()
        
    elif x < -tolerance:
        print("Moving left!")
        arlo.go_diff(leftSpeed*0.5, rightSpeed*0.5, 0, 1)
        sleep(0.2)
        arlo.stop()
       
    else: 
        print("Centered!!")
        arlo.go_diff(leftSpeed, rightSpeed, 1, 1) 
        sleep(0.2)
        arlo.stop()

# Estimate the distance from robot to observed landmark
def estimateLandmark(corners):
    cameraMatrix = np.array([[1414, 0,imageSize[0]/2],
                             [0, 1414,imageSize[1]/2],
                             [0,   0,   1]], dtype=np.float32)
    dist_coeffs = np.zeros((1, 5), dtype=np.float32)
    MARKER_SIZE = 0.145  

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_SIZE, cameraMatrix, dist_coeffs
    )

    while not atlandmark:
        travelLandmark(tvecs[0][0])
        searchLandmark(image_number)





def searchLandmark(image_number):
    detected = False
    # Search for landmark

    while (detected == False):
        # Capture frame
        image = cam.capture_array("main")

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        # Save frame
        filename = os.path.join(
            folder,
            f"image_{image_number:04d}.jpg"
        )

        cv2.imwrite(filename, image)
        print("Saved:", filename)
     
        # printing the captured result
        print(ids)


        image_number += 1

        # Wait 1 second
        time.sleep(1)

        if ids is not None:
            detected = True
            break    

        arlo.go_diff(leftSpeed, rightSpeed, 1, 0)
        sleep(0.2)
        arlo.go_diff(leftSpeed, rightSpeed, 0, 1)
        print(arlo.stop())
        sleep(1)

    print("Found landmark")
    estimateLandmark(corners)


searchLandmark(image_number)
