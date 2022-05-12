#!/usr/bin/env python

import cv2
from apriltag_perception import AprilTagPerception

# MiRo-E interface
import miro_ros_interface as mri

# MiRo-E parameters
import miro_constants as con
import rospy

# USAGE: Set MiRo in front of an example AprilTag
# (See here: https://april.eecs.umich.edu/software/apriltag)
# Enter the actual size of the tag and the distance from the camera, and ensure focal_length == None
# Run the script and note down the focal length given
# Enter the focal length and re-run the script; adjust MiRo's distance from the tag and check for accuracy

miro_per = mri.MiRoPerception()

# Set the actual size and distance of the AprilTag being used for calibration
# (Units don't matter as long as you use the same for both)
tag_size = 1
tag_distance = 2

# Enter a value here when you obtain a focal length to test your distance measurements are accurate
focal_length = None

atp = AprilTagPerception(size=tag_size, family='tag36h11')

while True and not rospy.core.is_shutdown():
	# Not yet sure if undistorted images work better or worse for distance estimation
	image = miro_per.caml_undistorted
	tags = atp.detect_tags(image)

	if tags is not None:
		for t, _ in enumerate(tags):
			atp.draw_box(image, tags[t], colour='green')
			atp.draw_center(image, tags[t], colour='red')

			# Output either focal length or estimated distance on the image
			if focal_length is not None:
				est_distance = (tag_size * focal_length) / tags[t].apparent_size
				cv2.putText(image, 'Distance: {0:.2f}cm'.format(est_distance), tuple(tags[t].corners[1]),
				            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
			else:
				est_focal_length = (tags[t].apparent_size * tag_distance) / tag_size
				cv2.putText(image, 'Focal length: {0:.2f}'.format(est_focal_length), tuple(tags[t].corners[1]),
				            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

				print('Estimated focal length: {}'.format(est_focal_length))

	else:
		print('No AprilTags in view!')

	cv2.imshow('AprilTag calibration', image)
	cv2.waitKey(5)
