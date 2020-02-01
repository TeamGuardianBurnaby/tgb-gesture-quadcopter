# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import cv2

class GestureControl:
	def __init__(self, trackerType = "csrt"):
		self.trackerType = trackerType

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
		self.tracker = OPENCV_OBJECT_TRACKERS[trackerType]()

	def run(self):
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
				(success, box) = self.tracker.update(frame)
		 
				# check to see if the tracking was a success
				if success:
					(x, y, w, h) = [int(v) for v in box]
					cv2.rectangle(frame, (x, y), (x + w, y + h),
						(0, 255, 0), 2)

					vectorX = x + w/2 - preX
					vectorY = y + h/2 - preY
					preX = x + w/2
					preY = y + w/2
		 
				# update the FPS counter
				fps.update()
				fps.stop()
		 
				# initialize the set of information we'll be displaying on
				# the frame
				info = [
					("Tracker", self.trackerType),
					("Success", "Yes" if success else "No"),
					("FPS", "{:.2f}".format(fps.fps())),
					("Centre of Box", "{}: {}".format(x + w/2, y + h/2)),
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
				self.tracker.init(frame, initBB)
				fps = FPS().start()

			# if the `q` key was pressed, break from the loop
			elif key == ord("q"):
				break
		 
		# if we are using a webcam, release the pointer
		vs.stop()
		 
		# close all windows
		cv2.destroyAllWindows()

if __name__ == "__main__":
	# construct the argument parser and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-t", "--tracker", type=str, default="csrt",
		help="OpenCV object tracker type")
	args = vars(ap.parse_args())

	gestureControl = GestureControl(args["tracker"])
	gestureControl.run()
