#!/usr/bin/env python
import os
import rospy # ROS Python interface
#from ros_mommy_daddy_msg.msg import RobotPub # ROS parent info subsciriber
from std_msgs.msg import Float32MultiArray

class ParentMiro(object):
    
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.robot_sound = False
        self.subscriber = rospy.Subscriber(
            #"/parent/parent_publisher", RobotPub, self.parent_cb
            "/parent/parent_publisher", Float32MultiArray, self.parent_cb
        )

    def parent_cb(self, action_data):
        #self.pos_x = action_data.pos_x
        #self.pos_y = action_data.pos_y
        self.pos_x = action_data.data[0]
        self.pos_y = action_data.data[1]
        if action_data.data[2] == 0:
            self.robot_sound = False
        else:
            self.robot_sound = True
        #self.robot_sound = action_data.robot_sound