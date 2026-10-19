import numpy as np
import os
import cv2 # Import the OpenCV library

# NOTE: This module must NOT open the camera (or the robot) at import time.
# main.py owns the single camera via camera.Camera, and the Pi only allows one
# PiCamera2 instance at a time. Opening a second one here caused
# "Failed to acquire camera: Device or resource busy". detectLandmark() now
# takes the shared camera object instead.

# scp_dest = 'konrad@172.20.10.3:/home/konrad/Desktop/rex/REX-students/Arlo/ex4'

imageSize  = (1640, 1232)
arloR     = 0.25
landmarkR = 0.15

folder = "camera_images"
os.makedirs(folder, exist_ok=True)

image_number = 0


def detectLandmark(cam):
    # `cam` is the shared camera.Camera owned by main.py.
    image = cam.get_next_frame()
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
    cv2.imwrite(filename, image)
    print("Saved:", filename)

    idss, tvecs, radius = estimateLandmark(corners, ids)
    
    obstaclelist = []

    for i in range(len(idss)):
        tvec = tvecs[i][0]
        x = float(tvec[0])
        z = float(tvec[2])
        #marker_id = int(idss[i][0])
        obstaclelist.append((x,z, radius))
    return obstaclelist

def estimateLandmark(corners, id):
    cameraMatrix = np.array([[1377, 0,imageSize[0]/2],
                             [0, 1377,imageSize[1]/2],
                             [0,   0,   1]], dtype=np.float32)
    dist_coeffs = np.zeros((1, 5), dtype=np.float32)
    MARKER_SIZE = 0.145  

    if id is None or len(corners) == 0:
        return np.empty((0, 1), dtype=int), np.empty((0, 1, 3), dtype=np.float32), landmarkR

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_SIZE, cameraMatrix, dist_coeffs
    )

    return id, tvecs, landmarkR





