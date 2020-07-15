#! /usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
import random

# prepare object points
nx = 8
ny = 6

# List of calibration images
images = glob.glob('RoboND-Camera-Calibration/calibration_small/Cal*.jpg')
# Select random image from the list
idx = random.randint(0,len(images))
# Read image
img = mpimg.imread(images[idx])
# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

# Find Chessboard corners
cornersPresent, corners = cv2.findChessboardCorners(gray,(nx,ny),None)

if cornersPresent:
    # Draw and display corners
    cv2.drawChessboardCorners(img, (nx,ny), corners, cornersPresent)
    plt.imshow(img)
    plt.savefig('chessboard_result.png')