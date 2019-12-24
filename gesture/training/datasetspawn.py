# From a set of raw pictures, spawn more samples to create a more comprehensive dataset
#
# This should not substitute for actual data!

import numpy as np
import cv2
from datetime import datetime

def spawnImages(originalImage):
    result = []

    if originalImage is None:
        print("Error! Unable to open image!")
        return result

    # Image properties
    (height, width) = originalImage.shape[:2]
    centre = (width / 2, height / 2)

    # Change Colourspace
    # grey = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)
    # result.append(grey)

    hsv = cv2.cvtColor(originalImage, cv2.COLOR_BGR2HSV)
    result.append(hsv)

    # Rotating image by 180 degrees
    M = cv2.getRotationMatrix2D(centre, 180, 1.0)
    rotated = cv2.warpAffine(originalImage, M, (width, height))
    result.append(rotated)

    # Change Colourspace after rotation
    # grey = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
    # result.append(grey)

    hsv = cv2.cvtColor(rotated, cv2.COLOR_BGR2HSV)
    result.append(hsv)
    
    return result
