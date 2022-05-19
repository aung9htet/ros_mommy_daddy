#!/usr/bin/env python
import os
import rospy # ROS Python interface
from ros_mommy_daddy_msg.msg import RobotPub # ROS parent info subscriber

class ParentMiro(object):
    
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.robot_sound = False
        self.subscriber = rospy.Subscriber(
            "/miro/parent_publisher", RobotPub, self.parent_cb
        )

    def parent_cb(self, action_data):
        self.pos_x = action_data.pos_x
        self.pos_y = action_data.pos_y
        self.robot_sound = action_data.robot_sound