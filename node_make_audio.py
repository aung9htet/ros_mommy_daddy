#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from std_msgs.msg import Int16MultiArray 

import miro2 as miro

class NodeMakeAudio(object):

    def __init__(self):

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.sound_pub = rospy.Publisher(
            topic_base_name + "/sensors/mics", Int16MultiArray, queue_size=0
        )
        self.sound_detail = [1000,50,250]

    def set_sound_cmd(self, freq = 0, volume = 0, duration = 0):
        self.mic_generate = Int16MultiArray()
        self.sound_detail = [freq, volume, duration]
        self.mic_generate.data = self.sound_detail

    def pub_sound(self):
        self.sound_pub.publish(self.mic_generate)
