#!/usr/bin/env python
import os
import rospy # ROS Python interface
from relationship_msgs.msg import RobotPub # ROS child info subsciriber

class ChildMiro(object):
    
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.robot_sound = False
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            topic_base_name + "/child_publisher", RobotPub, self.child_cb
        )

    def child_cb(self, action_data):
        self.pos_x = action_data.pos_x
        self.pos_y = action_data.pos_y
        self.robot_sound = action_data.robot_sound