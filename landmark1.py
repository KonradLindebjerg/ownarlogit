# This script opens the camera using picamera2
# and saves captured images.

import cv2
import time
import os
from pprint import *

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)


print("OpenCV version = " + cv2.__version__)


# Open a camera device for capturing
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

pprint(cam.camera_configuration())

time.sleep(1)


# Folder for images
folder = "camera_images"
os.makedirs(folder, exist_ok=True)

print("Saving images to:", folder)


# Capture images
image_number = 0

while True:

    # Capture frame
    image = cam.capture_array("main")

    # Save frame
    filename = os.path.join(
        folder,
        f"image_{image_number:04d}.jpg"
    )

    cv2.imwrite(filename, image)

    print("Saved:", filename)

    image_number += 1

    # Wait 1 second
    time.sleep(1)

