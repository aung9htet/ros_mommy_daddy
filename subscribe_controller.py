#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from central_controller.msg import Action # ROS action subscriber

class ActionMiro(object):
    
    def __init__(self):
        self.parent.action = 0
        self.child.action = 0
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/central_controller", Action, self.odom_cb)

    def action_cb(self, action_data):
        self.parent.action = action_data.action.child
        self.child.action = action_data.action.parent