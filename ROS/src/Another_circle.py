#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;

# Read image
#im = cv2.imread("imgs/5.jpg",cv2.IMREAD_GRAYSCALE)

# Setup SimpleBlobDetector parameters.


# Change thresholds
cap = cv2.VideoCapture("imgs/WIN_20190425_17_18_32_Pro.mp4")
params = cv2.SimpleBlobDetector_Params()

while(True):

	ret, im = cap.read()
	
	params.minThreshold = 100
	params.maxThreshold = 200


	# Filter by Area.
	params.filterByArea = True
	params.minArea = 400
	params.maxArea = 1500

	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.1

	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.87

	# Filter by Inertia
	params.filterByInertia = True
	params.minInertiaRatio = 0.01

	# Create a detector with the parameters
	detector = cv2.SimpleBlobDetector_create(params)


	# Detect blobs.
	keypoints = detector.detect(im)

	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
	# the size of the circle corresponds to the size of blob

	im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	# Show blobs
	#im_with_keypoints = cv2.resize(im_with_keypoints, (806, 605))
	cv2.imshow("Keypoints", im_with_keypoints)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()