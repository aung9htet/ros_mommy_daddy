#!/usr/bin/env python

import cv2
import rospy
import os
from math import radians 
from apriltag_perception import AprilTagPerception
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from subscribe_range import RangeMiro

# MiRo-E interface
import miro_ros_interface as mri

# MiRo-E parameters

import miro_constants as con

try:
    from miro2.lib import wheel_speed2cmd_vel
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel


class LocateTag:
    NODE_EXISTS = False
    TICK = 0.02
    CAM_FREQ = 1
    SLOW = 0.05
    FAST = 0.4
    DEBUG = False

    def __init__(self):
        print("Locating tag")
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print(topic_base_name)

        rospy.sleep(2.0)
        self.image_convert = CvBridge()


        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size = 1,
            tcp_nodelay = True
        )

        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size = 1,
            tcp_nodelay = True
        )


        
        

        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )


        self.input_camera = [None, None]
        self.new_frame = [False, False]
        self.tag = [None, None]
        self.frame_width = 640
        self.just_switched = True
        self.bookmark = 0
        self.found_tag = False

        self.status_code = 0

        self.tag_size = 1
        self.atp = AprilTagPerception(size=self.tag_size, family='tag36h11')
        self.sonar = RangeMiro()
        self.reset_head_pose()


    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, radians(34.0), 0.0, 0.0]
        t = 0
        while not rospy.core.is_shutdown():  # Check ROS is running
            # Publish state to neck servos for 1 sec
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break
        

    def drive(self, speed_l=0.1, speed_r=0.1):
        msg_cmd_vel = TwistStamped()
        wheel_speed = [speed_l, speed_r]
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta 
        self.pub_cmd_vel.publish(msg_cmd_vel)

    def callback_caml(self, ros_image):
        self.callback_cam(ros_image, 0)
    
    def callback_camr(self, ros_image):
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        try:
            image = self.image_convert.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.input_camera[index] = image
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            self.new_frame[index] = True
        except CvBridgeError as e:
            pass

    def drive(self, speed_l=0.1, speed_r=0.1):
        msg_cmd_vel = TwistStamped()
        wheel_speed = [speed_l, speed_r]
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta 
        self.vel_pub.publish(msg_cmd_vel)


    def lookForTag(self):

        if self.just_switched:
            self.just_switched = False

        for index in range(2):
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            self.tag[index] = self.atp.detect_tags(image)

        if not self.tag[0] or not self.tag[1]:
            self.drive(self.SLOW, -self.SLOW)
        else:
            self.drive(0, 0)
            print("MiRo has found the Tag!!")
            self.found_tag = True
            self.status_code = 2
            self.just_switched = True

    def approach(self):
        if self.just_switched:
            self.just_switched = False
        for index in range(2):
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            self.tag[index] = self.atp.detect_tags(image)
            #if in both cameras move forwards
            if self.tag[0] and self.tag[1]:
                self.drive(0.2, 0.2)
            elif self.tag[0] and not self.tag[1]:
                self.drive(0, 0.2)
            #if in right camera turn right
            elif not self.tag[0] and  self.tag[1]:
                self.drive(0.2, 0)
            elif self.sonar.range < 0.30:
                self.status_code = 3
            else:
                self.status_code = 0
                print("MiRo has lost the tag...")
                
    def resetStatusCode(self):
        self.status_code = 0

    def loop(self):
        self.counter = 0
        self.status_code = 0
        
        while not rospy.core.is_shutdown():
            if self.status_code == 1:
                print("looking for Tag")
                self.lookForTag()
            elif self.status_code == 2:
                print("MiRo is Approaching the tag")
                self.approach()
            elif self.status_code == 3:
                print("MiRo has Found the Tag!!")
                self.drive(0, 0)
                break
            else:
                self.status_code = 1

            rospy.sleep(self.TICK)


if __name__ == "__main__":
    main = LocateTag()
    main.loop()