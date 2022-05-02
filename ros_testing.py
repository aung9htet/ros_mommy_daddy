#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some other modules from within this package
from control_odom import MoveMiro

class RosTesting:
    def __init__(self):
        rospy.init_node('ros_testing')
        self.robot_movement = MoveMiro()

    def loop(self):
        lol = True
        while lol == True:
            self.robot_movement.set_move_cmd(linear=0.4)
            self.robot_movement.vel_publish()

if __name__ == '__main__':
    main = RosTesting()
    main.loop()