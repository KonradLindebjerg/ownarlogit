import robot
import numpy as np
import picamera2
import time
import os
import csv
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
