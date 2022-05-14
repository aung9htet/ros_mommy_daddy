#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import miro2 as miro
import os

# Import some other modules from within this package
from subscribe_odom import OdomMiro
from robot_explore import RobotExplore
from locate_april_tag import LocateTag
from subscribe_controller import ActionMiro
from node_detect_audio import NodeDetectAudio
#from ros_mommy_daddy_msg.msg import RobotPub
from std_msgs.msg import Float32MultiArray

class ChildNode:
    
    def __init__(self):
        # intialise node and required modules for project
        rospy.init_node('child_node')
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.child_pub = rospy.Publisher(
            #topic_base_name + '/child_publisher', RobotPub, queue_size= 0
            topic_base_name + '/child_publisher', Float32MultiArray, queue_size= 0
        )
        #self.robot_pub = RobotPub()
        self.robot_pub = Float32MultiArray()
        # subscribe to central controller
        self.sub_controller = ActionMiro()
        self.robot_explore = RobotExplore()
        self.robot_locate_tag = LocateTag()
        self.robot_odom = OdomMiro()
        self.robot_detect_audio = NodeDetectAudio()

         # variables to use
        # 0 is to approach and 1 is to explore
        self.action = self.sub_controller.child
        #self.receive_sound = False
        #self.robot_pub.pos_x = self.robot_odom.posx
        #self.robot_pub.pos_y = self.robot_odom.posy
        self.robot_pub.data = [self.robot_odom.posx, self.robot_odom.posy, 0]

    def child_node(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            # Update actions, odom
            self.action = self.sub_controller.child
            #self.robot_pub.pos_x = self.robot_odom.posx
            #self.robot_pub.pos_y = self.robot_odom.posy
            self.robot_pub.data[0] = self.robot_odom.posx
            self.robot_pub.data[1] = self.robot_odom.posy
            if self.action == 0:
                # look for tag
                self.robot_locate_tag.loop()
                # check if sound is received
                self.receive_sound = self.detect_sound()
                if self.receive_sound:
                    #self.robot_pub.robot_sound = True
                    self.robot_pub.data[2] = 1
                else:
                    #self.robot_pub.robot_sound = False
                    self.robot_pub.data[2] = 0
            else:
                # Exploration
                self.robot_explore.explore()
                #self.robot_pub.robot_sound = False
                self.robot_pub.data[2] = 0
            # publish
            self.child_pub.publish(self.robot_pub)
            rate.sleep()

    def detect_sound(self):
        if self.robot_detect_audio.freq > 2000:
            return True
        else:
            return False

if __name__ == '__main__':
    main = ChildNode()
    try:
        main.child_node()
    except rospy.ROSInterruptException:
        pass