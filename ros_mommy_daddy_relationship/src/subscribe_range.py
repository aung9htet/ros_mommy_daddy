#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from sensor_msgs.msg import Range   # ROS odometry subsciriber
from math import degrees

class RangeMiro(object):
    
    def __init__(self):
        self.field_of_view = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.range = 0.0
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(topic_base_name + "/sensors/sonar", Range, self.range_cb)

    def range_cb(self, odom_data):
        self.field_of_view = odom_data.field_of_view
        self.min_range = odom_data.min_range
        self.max_range = odom_data.max_range
        self.range = odom_data.range