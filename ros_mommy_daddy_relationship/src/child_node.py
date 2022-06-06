#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import miro2 as miro
import os
import numpy as np
from datetime import datetime

# Import some other modules from within this package
from subscribe_odom import OdomMiro
from robot_explore import RobotExplore
from locate_april_tag import LocateTag
from subscribe_controller import ActionMiro
from node_wag_tail import NodeWagTail
from node_detect_audio import NodeDetectAudio
from ros_mommy_daddy_msg.msg import RobotPub

class ChildNode:
    
    def __init__(self):
        # intialise node and required modules for project
        rospy.init_node('child_node')
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.child_pub = rospy.Publisher(
            topic_base_name + '/child_publisher', RobotPub, queue_size= 0
        )
        self.robot_pub = RobotPub()
        # subscribe to central controller
        self.sub_controller = ActionMiro()
        self.robot_explore = RobotExplore()
        self.robot_locate_tag = LocateTag()
        self.robot_odom = OdomMiro()
        self.robot_detect_audio = NodeDetectAudio()
        self.robot_wag = NodeWagTail()
        self.robot_found = False

        # variables to use
        # 0 is to approach and 1 is to explore
        self.action = self.sub_controller.child
        self.robot_pub.robot_sound = False
        self.robot_pub.pos_x = self.robot_odom.posx
        self.robot_pub.pos_y = self.robot_odom.posy
        self.start_time = rospy.get_rostime()
        self.odom_record = np.empty((0,2), float)
        rospy.on_shutdown(self.save_file)

    def child_node(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)

            # Update actions, odom
            self.action = self.sub_controller.child
            self.robot_pub.pos_x = self.robot_odom.posx
            self.robot_pub.pos_y = self.robot_odom.posy

            # record robot odom pos as a tuple
            robot_pos = np.array([np.array([self.robot_pub.pos_x, self.robot_pub.pos_y])])
            self.odom_record = np.append(self.odom_record, robot_pos, axis = 0)

            # proceed on actions
            if self.action == 0:
                # look for tag
                if self.robot_found == False:
                    self.robot_found = self.robot_locate_tag.loop()
                # check if sound is received
                self.robot_pub.robot_sound = self.detect_sound()
                if self.robot_pub.robot_sound:
                    self.robot_pub.robot_sound = True
                else:
                    self.robot_pub.robot_sound = False
                # get time for calculating sine graph
                time_elasped = rospy.get_rostime().secs - self.start_time.secs
                tail_value = np.sin((time_elasped*10)+((np.pi*2)/360))
                # wag tail
                self.robot_wag.set_wag_cmd(wag=tail_value)
                self.robot_wag.pub_wag()
            else:
                self.robot_found = False
                # Exploration
                self.robot_explore.explore()
                self.robot_pub.robot_sound = False
            # publish
            self.child_pub.publish(self.robot_pub)
            rospy.loginfo(os.getcwd())
            rate.sleep()

    def detect_sound(self):
        if self.robot_detect_audio.freq > 2000:
            return True
        else:
            return False

    def save_file(self):
        current_time = datetime.now()
        date_time = current_time.strftime("%m-%d-%Y_%H-%M-%S")
        file_name = "../mdk/catkin_ws/src/ros_mommy_daddy/ros_mommy_daddy_relationship/odom_data/child_odom_data_" + date_time + ".npy"
        np.save(file_name, np.asarray(self.odom_record))

if __name__ == '__main__':
    main = ChildNode()
    try:
        main.child_node()
    except rospy.ROSInterruptException: 
        pass