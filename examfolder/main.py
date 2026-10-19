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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lib import drive
from lib import robot

# What landmarks have been visited?
VisitedLandmark = {
    "L1": False,
    "L2": False,
    "L3": False,
    "L4": False
}

# Constants
EPS         = 0.05     # EPS for sleep time so arlo hsa time to catch up
SIGMA_THETA = 0.029320 # Sigma noise for when turning
SIGMA       = 2.0      # Sigma for driving distance








