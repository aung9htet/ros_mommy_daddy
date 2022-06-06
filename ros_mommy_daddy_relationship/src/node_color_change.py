#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import UInt32MultiArray

import miro2 as miro

class NodeColorChange(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.color_pub = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )

    # Making tail wag and ears move
    def set_color_cmd(self, red = 0.0, green = 0.0, blue = 0.0):
        self.color_change = UInt32MultiArray()
        self.color_detail = [red, green, blue]
        self.color_change.data = self.wag_detail

    def pub_color(self):
        self.color_pub.publish(self.color_change)
