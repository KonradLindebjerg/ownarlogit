import robot
import numpy as np
import picamera2
import time
import os
import csv
import subprocess
import cv2 # Import the OpenCV library
# Create a robot object and initialize
arlo = robot.Robot()

# Open a camera device for capturing
cam = cv2.VideoCapture(0)
scp_dest = 'konrad@172.20.10.3:/home/konrad/Desktop/rex/REX-students/Arlo/ex4'

leftSpeed  = 68
rightSpeed = 65
imageSize  = (1640, 1232)
FPS        = 30

cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000)


# Define radius' in m
arloR     = 0.225
landmarkR = 0.15



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



folder = "camera_images"
os.makedirs(folder, exist_ok=True)

print("Saving images to:", folder)
image_number = 0


def detectlandmarks():
    image = cam.capture_array("main")
    global image_number
    
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    
    
    
    
    
    # Save frame
    filename = os.path.join(
        folder,
        f"image_{image_number:04d}.jpg"
    )
    image_number += 1
    '''
    cv2.imwrite(filename, image)
    print("Saved:", filename)
    '''

    idss, tvecs = estimateLandmark(corners, ids)
    '''
    detection_folder = "landmarkdetections"
    os.makedirs(detection_folder, exist_ok=True)
    csv_path = os.path.join(detection_folder, "landmarks.csv")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        for i in range(len(idss)):
            tvec = tvecs[i][0]
            x = float(tvec[0])
            z = float(tvec[2])
            marker_id = int(idss[i][0])
            writer.writerow([x, z, marker_id])

    print("Saved landmark detections to:", csv_path)

    # Copy the CSV from the Pi to the laptop via scp.
    if scp_dest:
        subprocess.run(["scp", csv_path, scp_dest], check=True)
        print("Copied landmark detections to:", scp_dest)
    else:
        print("LAPTOP_SCP_DEST not set; skipping scp to laptop.")
    '''

def estimateLandmark(corners, id):
    cameraMatrix = np.array([[1414, 0,imageSize[0]/2],
                             [0, 1414,imageSize[1]/2],
                             [0,   0,   1]], dtype=np.float32)
    dist_coeffs = np.zeros((1, 5), dtype=np.float32)
    MARKER_SIZE = 0.145  

    if id is None or len(corners) == 0:
        return np.empty((0, 1), dtype=int), np.empty((0, 1, 3), dtype=np.float32)

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_SIZE, cameraMatrix, dist_coeffs
    )

    return id, tvecs


detectlandmarks()




def maplocation():



