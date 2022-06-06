#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from ros_mommy_daddy_msg.msg import Emotion # ROS emotion subscriber

class EmotionMiro(object):
    
    def __init__(self):
        self.ambivalent = 0
        self.avoidant = 0
        self.subscriber = rospy.Subscriber("/emotion_controller", Emotion, self.action_cb)

    def action_cb(self, action_data):
        self.ambivalent= action_data.ambivalent
        self.avoidant = action_data.avoidant