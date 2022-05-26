#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import miro2 as miro
import numpy as np
import os

# Import some other modules from within this package
from subscribe_odom import OdomMiro
from robot_explore import RobotExplore
from locate_april_tag import LocateTag
from subscribe_controller import ActionMiro
from node_make_audio import NodeMakeAudio
from node_wag_tail import NodeWagTail
from ros_mommy_daddy_msg.msg import RobotPub

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

        # variables to use
        # 0 is to approach and 1 is to explore
        self.action = self.sub_controller.parent
        self.produce_sound = False
        self.robot_pub.pos_x = self.robot_odom.posx
        self.robot_pub.pos_y = self.robot_odom.posy
        self.control_tail = 0
        self.start_time = rospy.get_rostime()
        self.robot_found = False

    def parent_node(self):
        rate = rospy.Rate(500)

        while not rospy.is_shutdown():            
            # update actions, odom
            self.control_tail += 1
            self.action = self.sub_controller.parent
            self.robot_pub.pos_x = self.robot_odom.posx
            self.robot_pub.pos_y = self.robot_odom.posy
            # sound is not made at start
            self.robot_pub.robot_sound = False
            rospy.loginfo(self.action)
            if self.action == 0:
                rospy.loginfo("wagging")
                # look for tag
                if self.robot_found == False:
                    self.robot_found = self.robot_locate_tag.loop()
                # sound will start to be made and thus it is true
                self.robot_pub.robot_sound = True
                # produce sound
                self.robot_make_audio.produce_sound(freq= 2000, volume=255, duration=25)
                # get time for calculating sine graph
                time_elasped = rospy.get_rostime().secs - self.start_time.secs
                tail_value = np.sin((time_elasped*10)+((np.pi*2)/360))
                # wag tail
                self.robot_wag.set_wag_cmd(wag=tail_value)
                self.robot_wag.pub_wag()
            else:
                self.robot_found = False
            # Exploration
                rospy.loginfo("exploring")
                self.robot_explore.explore()
                self.robot_pub.robot_sound = False
                # publish
                rospy.loginfo("publishing")
                self.parent_pub.publish(self.robot_pub)
                rate.sleep()
    
if __name__ == '__main__':
    main = ParentNode()
    try:
        main.parent_node()
    except rospy.ROSInterruptException:
        pass
