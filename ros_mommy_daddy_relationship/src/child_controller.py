#!/usr/bin/env python
import os
import rospy # ROS Python interface
from ros_mommy_daddy_msg.msg import RobotPub # ROS child info subscriber

class ChildMiro(object):
    
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.robot_sound = False
        self.subscriber = rospy.Subscriber(
            "/david/child_publisher", RobotPub, self.child_cb
        )

    def child_cb(self, action_data):
        self.pos_x = action_data.pos_x
        self.pos_y = action_data.pos_y
        self.robot_sound = action_data.robot_sound