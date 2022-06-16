#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Float32MultiArray

import miro2 as miro

class NodeWagTail(object):

    def __init__(self):
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.wag_pub = rospy.Publisher(
            topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

    # Making tail wag and ears move
    def set_wag_cmd(self, droop=0.0, wag=1.0, eyel=0.0, eyer=0.0, earl=1.0, earr=1.0):
        self.wag_tail = Float32MultiArray()
        self.wag_detail = [droop, wag, eyel, eyer, earl, earr]
        self.wag_tail.data = self.wag_detail

    def pub_wag(self):
        self.wag_pub.publish(self.wag_tail)
