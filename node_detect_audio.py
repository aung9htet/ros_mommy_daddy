#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Int16MultiArray

import miro2 as miro

class NodeDetectAudio(object):

	def __init__(self):

		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
		self.sound_subscriber = rospy.Subscriber(topic_base_name + "/sensors/mics", Int16MultiArray, self.detect_sound_cmd)
		self.freq = 0
		self.volume = 0
		self.duration = 0

	def detect_sound_cmd(self, sound_data):
		sound_detail = sound_data.data
		self.freq = sound_detail[0]
		self.volume = sound_detail[1]
		self.duration = sound_detail[2]