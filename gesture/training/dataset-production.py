# See https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html

import numpy as np
import cv2
from datetime import datetime

cap = cv2.VideoCapture(0)

# Verify input shape
width = cap.get(3)
height = cap.get(4)
fps = cap.get(5)
print(width, height, fps)

# Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,720))

while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()

  # Write the frame
  timestamp = datetime.timestamp(datetime.now())
  filename = str(timestamp) + ".jpg"
  cv2.imwrite("raw-images/" + filename, frame) 

  # Display the resulting frame
  cv2.imshow('frame',frame)
  if cv2.waitKey(1) & 0xFF == ord('q'):
      break

# Release everything if job is finished
cv2.destroyAllWindows()
cap.release()
