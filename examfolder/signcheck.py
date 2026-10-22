"""30-second sign check. Run on the robot before main.py:  python3 signcheck.py

1. Hold an ArUco marker to the robot's LEFT  -> printed angle must be POSITIVE.
   Wrong? flip BEARING_SIGN in camera.py.
2. Then the robot turns +45 deg               -> it must turn LEFT (counter-clockwise from above).
   Wrong? flip TURN_DIR in lib/drive.py.
"""
import os
import sys
import time
import camera

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib import drive

cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)

print("Step 1: hold a marker to the robot's LEFT (should print a POSITIVE angle)")
end = time.time() + 8
while time.time() < end:
    ids, dists, angles = cam.detect_aruco_objects(cam.get_next_frame())
    if ids is not None:
        for i in range(len(ids)):
            side = "LEFT" if angles[i] > 0 else "RIGHT"
            print(f"  id={ids[i]}  angle={angles[i] * 57.2958:+.1f} deg  -> code thinks marker is {side}")

print("Step 2: turning +45 deg, robot should turn LEFT")
drive.turn(45, [])
