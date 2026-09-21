# ======= TODO list =======
#
# Vi skal kalibere kameraet så vi får Camera matrix (som jeg tror skal bruge focal length som vi udregnede før) og distortion matrix som jeg ikke ved noget om
#
#
# === IGNORE === 
# if len(corners) > 0:
#       for i in range(0, len(ids)):
#           # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
#           rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
#                                                                       distortion_coefficients)
#           # Draw a square around the markers
#           cv2.aruco.drawDetectedMarkers(frame, corners) 
#
#           # Draw Axis
#           cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


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



# Moving towards the found lander 

searchLandmark(image_number)

