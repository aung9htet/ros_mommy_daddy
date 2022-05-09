#!/user/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import numpy as np
import time

# Import some other modules from within this package
from control_odom import MoveMiro
from subscribe_odom import OdomMiro
from subscribe_range import RangeMiro

# This module will focus on the ros exploring randomly or return back to position
class RobotExplore:
    
    def __init__(self):
        # initialise node and required modules for project
        rospy.init_node('robot_explore')
        self.robot_movement = MoveMiro()
        self.robot_odom = OdomMiro()
        self.robot_range = RangeMiro()

        # set position for marking mother position
        self.marked_x = self.robot_odom.posx
        self.marked_y = self.robot_odom.posy

        # identify how near miro is to a wall perpendicular to its vision
        self.wall_distance = self.robot_range.range
        
        # set variables for project
        self.speed = 0.4
        self.min_wall = 0.2
        self.turn_angle = 0
        self.start_time = rospy.get_rostime()
        self.start_turn = 0
        self.explore_robot = True

    # check whether to turn
    def turn_robot(self):
        # check if robot is in the middle of turning
        if self.explore_robot == False:
            if ((rospy.get_rostime().secs - self.start_turn.secs) % 60) == 0:
                self.explore_robot = True
            print("holy crap")
            return True
        # check if the wall is too near
        if self.wall_distance < self.min_wall:
            self.start_turn = rospy.get_rostime()
            self.explore_robot = False
            print("lol")
            return True
        else:
            return False
    
    # to explore randomly around
    def explore(self):
        # get time elasped
        time_elasped = rospy.get_rostime().secs - self.start_time.secs
        self.wall_distance = self.robot_range.range
        # set maximum angle of turns to be 10 degrees
        if self.turn_robot() == False:
            if ((time_elasped % 2) == 0):
                max_angle_turn = ((2 * np.pi)/360) * 80
                rand_turn = np.random.randint(-np.ceil(max_angle_turn * 10), np.ceil(max_angle_turn * 10))/10
                self.turn_angle = rand_turn
            self.robot_movement.set_move_cmd(linear=self.speed, angular=self.turn_angle)
            self.robot_movement.vel_publish()
        else:
            angle_turn = (np.pi/2)/3 * 2
            self.robot_movement.set_move_cmd(linear=0, angular=angle_turn)
            self.robot_movement.vel_publish()
                
    def testing(self):
        lol = True
        while lol == True:
            self.explore()


if __name__ == '__main__':
    main = RobotExplore()
    main.testing()      
