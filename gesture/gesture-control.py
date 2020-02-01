# # import the necessary packages
# from keras.models import load_model
# from pyimagesearch.centroidTracker import CentroidTracker
# from imutils.video import VideoStream
# import numpy as np
# import argparse
# import imutils
# import pickle
# import time
# import cv2

# # construct the argument parser and parse the arguments
# ap = argparse.ArgumentParser()
# # ap.add_argument("-i", "--image", required=True,
# # 	help="path to input image we are going to classify")
# ap.add_argument("-m", "--model", required=True,
# 	help="path to trained Keras model")
# ap.add_argument("-l", "--labelbin", required=True,
# 	help="path to label binarizer")
# # ap.add_argument("-w", "--width", type=int, default=28,
# # 	help="target spatial dimension width")
# # ap.add_argument("-e", "--height", type=int, default=28,
# # 	help="target spatial dimension height")
# # ap.add_argument("-f", "--flatten", type=int, default=-1,
# # 	help="whether or not we should flatten the image")
# args = vars(ap.parse_args())


# # initialize our centroid tracker and frame dimensions
# ct = CentroidTracker()
# (H, W) = (None, None)

# # load the model and label binarizer
# print("[INFO] loading network and label binarizer...")
# model = load_model(args["model"])
# lb = pickle.loads(open(args["labelbin"], "rb").read())

# # initialize the video stream and allow the camera sensor to warmup
# print("[INFO] starting video stream...")
# vs = VideoStream(src=0).start()
# time.sleep(2.0)

# # loop over the frames from the video stream
# while True:
# 	# read the next frame from the video stream and resize it
# 	frame = vs.read()
# 	frame = imutils.resize(frame, width=400)
 
# 	# if the frame dimensions are None, grab them
# 	if W is None or H is None:
# 		(H, W) = frame.shape[:2]
 
# 	# construct a blob from the frame, pass it through the network,
# 	# obtain our output predictions, and initialize the list of
# 	# bounding box rectangles
# 	blob = cv2.dnn.blobFromImage(frame, 1.0, (W, H),
# 		(104.0, 177.0, 123.0))
# 	net.setInput(blob)
# 	detections = net.forward()
# 	rects = []

# 	# loop over the detections
# 	for i in range(0, detections.shape[2]):
# 		# filter out weak detections by ensuring the predicted
# 		# probability is greater than a minimum threshold
# 		if detections[0, 0, i, 2] > args["confidence"]:cl
# 			# compute the (x, y)-coordinates of the bounding box for
# 			# the object, then update the bounding box rectangles list
# 			box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
# 			rects.append(box.astype("int"))
 
# 			# draw a bounding box surrounding the object so we can
# 			# visualize it
# 			(startX, startY, endX, endY) = box.astype("int")
# 			cv2.rectangle(frame, (startX, startY), (endX, endY),
# 				(0, 255, 0), 2)

# 	# update our centroid tracker using the computed set of bounding
# 	# box rectangles
# 	objects = ct.update(rects)
 
# 	# loop over the tracked objects
# 	for (objectID, centroid) in objects.items():
# 		# draw both the ID of the object and the centroid of the
# 		# object on the output frame
# 		text = "ID {}".format(objectID)
# 		cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
# 			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
# 		cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
 
# 	# show the output frame
# 	cv2.imshow("Frame", frame)
# 	key = cv2.waitKey(1) & 0xFF
 
# 	# if the `q` key was pressed, break from the loop
# 	if key == ord("q"):
# 		break
 
# # do a bit of cleanup
# cv2.destroyAllWindows()
# vs.stop()

## Temporary workaround
# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--tracker", type=str, default="csrt",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

# extract the OpenCV version info
(major, minor) = cv2.__version__.split(".")[:2]
 
# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())
 
# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:
	# initialize a dictionary that maps strings to their corresponding
	# OpenCV object tracker implementations
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}
 
	# grab the appropriate object tracker using our dictionary of
	# OpenCV object tracker objects
	tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
 
# initialize the bounding box coordinates of the object we are going
# to track
initBB = None

# if a video path was not supplied, grab the reference to the web cam
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(1.0)
 
# initialize the FPS throughput estimator
fps = None
preX = 0.0
preY = 0.0

# loop over frames from the video stream
while True:
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame
 
	# check to see if we have reached the end of the stream
	if frame is None:
		break
 
	# resize the frame (so we can process it faster) and grab the
	# frame dimensions
	frame = imutils.resize(frame, width=500)
	(H, W) = frame.shape[:2]

		# check to see if we are currently tracking an object
	if initBB is not None:
		# grab the new bounding box coordinates of the object
		(success, box) = tracker.update(frame)
 
		# check to see if the tracking was a success
		if success:
			(x, y, w, h) = [int(v) for v in box]
			cv2.rectangle(frame, (x, y), (x + w, y + h),
				(0, 255, 0), 2)

			vectorX = x + w/2 - preX
			vectorY = y + w/2 - preY
			preX = x + w/2
			preY = y + w/2
 
		# update the FPS counter
		fps.update()
		fps.stop()
 
		# initialize the set of information we'll be displaying on
		# the frame
		info = [
			("Tracker", args["tracker"]),
			("Success", "Yes" if success else "No"),
			("FPS", "{:.2f}".format(fps.fps())),
			("Centre of Box", "{}: {}".format(x + w/2, y + w/2)),
			("Vector of Box", "{:.2f}: {:.2f}".format(vectorX, vectorY)),
		]
 
		# loop over the info tuples and draw them on our frame
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
				cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		initBB = cv2.selectROI("Frame", frame, fromCenter=False,
			showCrosshair=True)

		# start OpenCV object tracker using the supplied bounding box
		# coordinates, then start the FPS throughput estimator as well
		tracker.init(frame, initBB)
		fps = FPS().start()

	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		initBB = cv2.selectROI("Frame", frame, fromCenter=False,
			showCrosshair=True)
 
		# start OpenCV object tracker using the supplied bounding box
		# coordinates, then start the FPS throughput estimator as well
		tracker.init(frame, initBB)
		fps = FPS().start()

	# if the `q` key was pressed, break from the loop
	elif key == ord("q"):
		break
 
# if we are using a webcam, release the pointer
vs.stop()
 
# close all windows
cv2.destroyAllWindows()
