#!/usr/bin/env python
import os
import rospy    # ROS Python interface
from nav_msgs.msg import Odometry   # ROS odometry subsciriber
from tf.transformations import euler_from_quaternion
from math import degrees

class ColorDetectMiro(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        if yaw<0:
            self.yaw = self.round(degrees(yaw), 4)+360
        else:
            self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

    def __init__(self):
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size=1,
            tcp_nodelay=True,
        )

    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)