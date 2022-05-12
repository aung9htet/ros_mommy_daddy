#!/user/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import miro2 as miro

# Import some other modules from within this package
from subscribe_odom import OdomMiro
from robot_explore import RobotExplore
from locate_april_tag import LocateTag
from subscribe_controller import ActionMiro

# Class for creating object when publishing
class ParentPub:
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.robot_sound = False

class ParentNode:

    def __init__(self):
        # intialise node and required modules for project
        rospy.init('parent_node')
        # publish the parent pub object
        self.parent_pub = rospy.Publisher('parent_publisher', ParentPub, queue_size= 10)
        self.robot_pub = ParentPub()
        # subscribe to central controller
        self.sub_controller = ActionMiro()
        self.robot_explore = RobotExplore()
        self.robot_locate_tag = LocateTag()
        self.robot_odom = OdomMiro()

        # variables to use
        # 0 is to approach and 1 is to explore
        self.action = self.sub_controller.parent.action
        self.produce_sound = False
        self.robot_pub.pos_x = self.robot_odom.posx
        self.robot_pub.pos_y = self.robot_odom.posy

    def parent_node(self):
        while rospy.is_shutdown():
            # Update actions, odom
            self.action = self.sub_controller.parent.action
            self.robot_pub.pos_x = self.robot_odom.posx
            self.robot_pub.pos_y = self.robot_odom.posy
            if self.action == 0:
                # check if MiRo is near the robot
                if self.robot_locate_tag.status_code == 4:
                    # Update if sound is being given
                    # Add your method to produce sound here
                    self.robot_pub.robot_sound = True
                else:
                    # Approach towards the other MiRo using the tag
                    self.robot_locate_tag.approach_miro()
                    self.robot_pub.robot_sound = False
            else:
                # Exploration
                self.robot_explore.explore()
                self.robot_pub.robot_sound = False
            # publish
            self.robot_pub.pub(self.robot_pub)
    
    if __name__ == '__main__':
    try:
        parent_node()
    except rospy.ROSInterruptException:
        pass