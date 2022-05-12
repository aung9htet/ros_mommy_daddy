import cv2
import numpy as np

# MiRo-E interface
import miro_ros_interface as mri

# USAGE: Set MiRo in front of a checkerboard calibration screen
# (Get one from here: https://markhedleyjones.com/projects/calibration-checkerboard-collection)
# Set the number of rows and columns to the number of *internal* corners visible by MiRo (i.e. number of squares - 1)
# Run the script and adjust MiRo's position until a rainbow calibration image is shown alongside a undistorted image
# (Example of the rainbow pattern: https://docs.opencv.org/3.3.1/dc/dbb/tutorial_py_calibration.html)
# Fill as much of the camera view as possible with a detected checkerboard for the best results
# Note down the MTX and DIST array values and use them in the AprilTag perception (or elsewhere)

miro_per = mri.MiRoPerception()

# This is adapted from https://docs.opencv.org/3.3.1/dc/dbb/tutorial_py_calibration.html
# TODO: Find out significance of hardcoded numbers here

# Termination criteria (see https://docs.opencv.org/master/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Chessboard internal dimensions
col = int(8)
row = int(6)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0),..., (6,5,0)
objp = np.zeros((row * col, 3), np.float32)
objp[:, :2] = np.mgrid[0:col, 0:row].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []      # 3D point in real world space
imgpoints = []      # 2D points in image plane

if miro_per.caml is not None:
	# Allow user to adjust MiRo to find chessboard
	# CURRENTLY SET UP FOR LEFT EYE ONLY
	# TODO: Set up a way to cleanly exit the calibration rather than just waiting for 200 cycles or quitting
	for i in range(200):
		# Get MiRo camera image
		cam_l = np.array(miro_per.caml)
		# cam_r = np.array(miro_per.camr)

		# Convert images to grayscale for chessboard detection
		gray_l = cv2.cvtColor(cam_l, cv2.COLOR_BGR2GRAY)
		# gray_r = cv2.cvtColor(cam_r, cv2.COLOR_BGR2GRAY)

		# Find the chessboard corners
		cb_found, corners = cv2.findChessboardCorners(gray_l, (col, row), None)

		# If found, add object points, image points (after refining them)
		if cb_found:
			objpoints.append(objp)

			cv2.cornerSubPix(gray_l, corners, (11, 11), (-1, -1), criteria)
			imgpoints.append(corners)

			# Draw and display the corners
			cv2.drawChessboardCorners(cam_l, (col, row), corners, cb_found)

			# Return the camera matrix, distortion coefficients, rotation and translation vectors
			ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_l.shape[::-1], None, None)

			# Undistort image
			mod_l = cv2.undistort(cam_l, mtx, dist, None)
			cv2.imshow('Undistorted image', mod_l)

			print('MTX: {}'.format(mtx))
			print('DIST: {}'.format(dist))

		cv2.imshow('Image', cam_l)
		cv2.waitKey(5)
	else:
		print('No image detected; are your ROS settings correct?')
