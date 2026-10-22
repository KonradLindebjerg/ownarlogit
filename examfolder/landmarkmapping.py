import numpy as np
import os
import cv2 # Import the OpenCV library
import camera  # only for BEARING_SIGN; does not open the camera

# NOTE: This module must NOT open the camera (or the robot) at import time.
# main.py owns the single camera via camera.Camera, and the Pi only allows one
# PiCamera2 instance at a time. Opening a second one here caused
# "Failed to acquire camera: Device or resource busy". detectLandmark() now
# takes the shared camera object instead.

# scp_dest = 'konrad@172.20.10.3:/home/konrad/Desktop/rex/REX-students/Arlo/ex4'

imageSize  = (1640, 1232)
# Radii in centimeters (the map/RRT convention used by main.py).
arloR     = 25.0
landmarkR = 15.0

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
        # estimatePoseSingleMarkers returns tvecs in the same unit as
        # MARKER_SIZE (meters); convert to centimeters for the map/RRT frame.
        # rotate_obstacles() expects x = robot's RIGHT. Use the same sign as
        # camera.py so obstacles aren't mirrored left/right.
        x = -camera.BEARING_SIGN * float(tvec[0]) * 100
        z = float(tvec[2]) * 100
        marker_id = int(idss[i][0])
        obstaclelist.append((x,z, radius, marker_id))
    return obstaclelist

def estimateLandmark(corners, id):
    cameraMatrix = np.array([[1200, 0,imageSize[0]/2],
                             [0, 1200,imageSize[1]/2],
                             [0,   0,   1]], dtype=np.float32)
    dist_coeffs = np.zeros((1, 5), dtype=np.float32)
    MARKER_SIZE = 0.15  # [m] must match camera.py's arucoMarkerLength

    if id is None or len(corners) == 0:
        return np.empty((0, 1), dtype=int), np.empty((0, 1, 3), dtype=np.float32), landmarkR

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_SIZE, cameraMatrix, dist_coeffs
    )

    return id, tvecs, landmarkR





