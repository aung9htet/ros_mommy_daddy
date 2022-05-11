#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Int16MultiArray

import miro2 as miro

class NodeDetectAudio(object):

	def __init__(self):

		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
		self.sound_sub = rospy.Subscriber(
			topic_base_name + "/control/tone", Int16MultiArray, queue_size=0
		)
		self.sound_detected = False
		self.care_sound = [1000,50,250]

		def detect_sound_cmd(self, freq = 0, volume = 0, duration = 0):
			self.mic_receive = Int16MultiArray()
			self.sound_detail = [freq, volume, duration]
			
			if self.sound_detail == self.care_sound:
				self.sound_detected = True