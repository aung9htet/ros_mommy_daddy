##!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Int16MultiArray

import miro2 as miro

class NodeDetectAudio(object):

	def __init__(self):

		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
		self.detect_sound_pub = rospy.Publisher(
			topic_base_name + "/control/tone", Int16MultiArray, queue_size=0
		)
		self.care_sound = [0,0,0]

	def detect_sound_cmd(self, freq = 0, volume = 0, duration = 0):
		self.mic_receive = Int16MultiArray()
		self.sound_detail = [freq, volume, duration]
		self.mic_receive.data = self.sound_detail

	def pub_detect_sound(self):
		self.detect_sound_pub.publish(self.mic_receive)
