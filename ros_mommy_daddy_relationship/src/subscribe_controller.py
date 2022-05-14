#!/usr/bin/env python
import os
import rospy    # ROS Python interface
#from ros_mommy_daddy_msg.msg import Action # ROS action subscriber
from std_msgs.msg import Int16MultiArray

class ActionMiro(object):
    
    def __init__(self):
        self.parent = 0
        self.child = 0
        #self.subscriber = rospy.Subscriber("/central_controller", Action, self.action_cb)
        self.subscriber = rospy.Subscriber("/central_controller", Int16MultiArray, self.action_cb)

    def action_cb(self, action_data):
        self.parent= action_data.data[1]
        self.child = action_data.data[0]