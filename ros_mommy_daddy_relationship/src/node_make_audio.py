#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from std_msgs.msg import UInt16MultiArray 

import miro2 as miro

class NodeMakeAudio(object):

    def __init__(self):

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.sound_pub = rospy.Publisher(
            topic_base_name + "/control/tone", UInt16MultiArray, queue_size=0
        )
        self.sound_detail = [1000,50,250]

    def produce_sound(self, freq = 0, volume = 0, duration = 0):
        self.speaker_generate = UInt16MultiArray()
        self.sound_detail = [freq, volume, duration]
        self.speaker_generate.data = self.sound_detail
        self.sound_pub.publish(self.speaker_generate)
