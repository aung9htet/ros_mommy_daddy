#!/usr/bin/env python
import os
import rospy # ROS Python interface
from child_publisher.msg import ParentPub # ROS parent info subsciriber

class ParentMiro(object):
    
    def __init__(self):
        self.odom.pos_x = 0
        self.odom.pos_y = 0
        self.robot_sound = False
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            topic_base_name + "/child_publisher", ParentPub, self.parent_cb
        )

    def child_cb(self, action_data):
        self.odom.pos_x = action_data.pos_x
        self.odom.pos_y = action_data.pos_y
        self.robot_sound = action_data.robot_sound