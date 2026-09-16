from time import sleep

import robot
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

counter=0
image_number = 0

# Search for landmark
while (counter < 10):
    # Capture frame
    image = cam.capture_array("main")

    arlo.go_diff(leftSpeed, rightSpeed, 1, 0)
    sleep(0.2)
    arlo.go_diff(leftSpeed, rightSpeed, 0, 1)
    print(arlo.stop())
    sleep(1)

    # Save frame
    filename = os.path.join(
        folder,
        f"image_{image_number:04d}.jpg"
    )

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

    print(corners)

    cv2.imwrite(filename, image)

    print("Saved:", filename)

    image_number += 1

    # Wait 1 second
    time.sleep(1)
    counter += 1
