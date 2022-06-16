#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import miro2 as miro
import numpy as np
import os
from datetime import datetime

# Import some other modules from within this package
from subscribe_odom import OdomMiro
from robot_explore import RobotExplore
from locate_april_tag import LocateTag
from subscribe_controller import ActionMiro
from node_make_audio import NodeMakeAudio
from node_wag_tail import NodeWagTail
from ros_mommy_daddy_msg.msg import RobotPub
from node_color_change import NodeColorChange

class ParentNode:

    def __init__(self):
        # intialise node and required modules for project
        rospy.init_node('parent_node')
        # publish the parent pub object
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.parent_pub = rospy.Publisher(
            topic_base_name + '/parent_publisher', RobotPub, queue_size= 0
        )
        self.robot_pub = RobotPub()
        # subscribe to central controller
        self.sub_controller = ActionMiro()
        self.robot_explore = RobotExplore()
        self.robot_locate_tag = LocateTag()
        self.robot_odom = OdomMiro()
        self.robot_make_audio = NodeMakeAudio()
        self.robot_wag = NodeWagTail()
        self.robot_color = NodeColorChange()

        # variables to use
        # 0 is to approach and 1 is to explore
        self.action = self.sub_controller.parent
        self.produce_sound = False
        self.robot_pub.pos_x = self.robot_odom.posx
        self.robot_pub.pos_y = self.robot_odom.posy
        self.control_tail = 0
        self.start_time = rospy.get_rostime()
        self.robot_found = False
        self.odom_record = np.empty((0,3), float)
        rospy.on_shutdown(self.save_file)

    def parent_node(self, explore_tag = False):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():            
            # update actions, odom
            self.control_tail += 1
            self.action = self.sub_controller.parent
            self.robot_pub.pos_x = self.robot_odom.posx
            self.robot_pub.pos_y = self.robot_odom.posy

            # color change here
            self.robot_color.set_color_cmd(red = self.sub_controller.emotional_distance*255.0, green = (1 - self.sub_controller.emotional_distance)*255.0)
            self.robot_color.pub_color()

            # sound is not made at start
            rospy.loginfo(self.action)
            if self.action == 0:
                rospy.loginfo("wagging")
                # look for tag
                self.robot_make_audio.produce_sound(freq= 2000, volume=255, duration=25)
                self.robot_pub.robot_sound = True
                if self.robot_found == False:
                    self.robot_found = self.robot_locate_tag.loop(0)
                else:
                    # sound will start to be made and thus it is true
                    # produce sound
                    # get time for calculating sine graph
                    time_elasped = rospy.get_rostime().secs - self.start_time.secs
                    tail_value = np.sin((time_elasped*10)+((np.pi*2)/360))
                    # wag tail
                    self.robot_wag.set_wag_cmd(wag=tail_value)
                    self.robot_wag.pub_wag()
            else:
                if explore_tag == False:
                    self.robot_found = False
                    # Exploration
                    self.robot_explore.min_wall = 0.2
                    self.robot_explore.explore()
                    self.robot_pub.robot_sound = False
                else:
                    response = self.robot_explore.explore_tags()
                    if response == True:
                        # do something
                        time_elasped = rospy.get_rostime().secs - self.start_time.secs
                        while (time_elasped % 4) > 0:
                            # get time for calculating sine graph
                            time_elasped = rospy.get_rostime().secs - self.start_time.secs
                            tail_value = np.sin((time_elasped*10)+((np.pi*2)/360))
                            # wag tail
                            self.robot_wag.set_wag_cmd(wag=tail_value)
                            self.robot_wag.pub_wag()
            
            # publish
            self.parent_pub.publish(self.robot_pub)
            rospy.loginfo(os.getcwd())

            # record robot odom pos as an np array
            if self.robot_found == True:
                robot_pos = np.array([np.array([self.robot_pub.pos_x, self.robot_pub.pos_y, 1])])
            else:
                robot_pos = np.array([np.array([self.robot_pub.pos_x, self.robot_pub.pos_y, 0])])
            self.odom_record = np.append(self.odom_record, robot_pos, axis = 0)

            rate.sleep()

    def save_file(self):
        current_time = datetime.now()
        date_time = current_time.strftime("%m-%d-%Y_%H-%M-%S")
        file_name = "../mdk/catkin_ws/src/ros_mommy_daddy/ros_mommy_daddy_relationship/odom_data/parent_odom_data_" + date_time + ".npy"
        np.save(file_name, np.asarray(self.odom_record))

if __name__ == '__main__':
    main = ParentNode()
    try:
        main.parent_node()
    except rospy.ROSInterruptException:
        pass
