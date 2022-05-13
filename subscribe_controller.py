#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from relationship_msgs.msg import Action # ROS action subscriber

class ActionMiro(object):
    
    def __init__(self):
        self.parent = 0
        self.child = 0
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/central_controller", Action, self.action_cb)

    def action_cb(self, action_data):
        self.parent= action_data.child
        self.child = action_data.parent