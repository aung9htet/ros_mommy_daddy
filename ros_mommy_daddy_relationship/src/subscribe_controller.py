#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from ros_mommy_daddy_msg.msg import Action # ROS action subscriber

class ActionMiro(object):
    
    def __init__(self):
        self.parent = 0
        self.child = 0
        self.emotional_distance = 0
        self.physical_distance = 0
        self.child_need = 0
        self.parent_need = 0
        self.subscriber = rospy.Subscriber("/central_controller", Action, self.action_cb)

    def action_cb(self, action_data):
        self.parent= action_data.parent
        self.child = action_data.child
        self.emotional_distance = action_data.emotional_distance
        self.physical_distance = action_data.physical_distance
        self.child_need = action_data.child_need
        self.parent_need = action_data.parent_need