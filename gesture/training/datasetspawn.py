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

    # Insert resize parameters, as long as it is greater than 100px
    tempWidth = 0.5*width
    resizeWidths = []

    while tempWidth > 100:
        resizeWidths.append(int(tempWidth))
        tempWidth = tempWidth*0.5
    resizeWidths.append(100)

    # Loop through the different resize widths
    for resizeWidth in resizeWidths:

        # Resizing images from original resolution to a width max of 100px (keep aspect ratio)
        ratio = resizeWidth/width
        dim = (resizeWidth, int(height * ratio))
        resized = cv2.resize(originalImage, dim, interpolation = cv2.INTER_AREA)
        result.append(resized)

        # Change Colourspace
        grey = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        result.append(grey)

        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        result.append(hsv)

        # Rotating image by 180 degrees
        (newHeight, newWidth) = resized.shape[:2]
        centre = (newWidth / 2, newHeight / 2)

        M = cv2.getRotationMatrix2D(centre, 180, 1.0)
        rotated = cv2.warpAffine(resized, M, (newWidth, newHeight))
        result.append(rotated)

        # Change Colourspace after rotation
        grey = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
        result.append(grey)

        hsv = cv2.cvtColor(rotated, cv2.COLOR_BGR2HSV)
        result.append(hsv)
