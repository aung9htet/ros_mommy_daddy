#!/user/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import numpy as np
import miro2 as miro
import os

# import some other modules from within this package
from child_controller import ChildMiro
from parent_controller import ParentMiro
# class for actions
class Action:
    def __init__(self):
        self.action.child = 0
        self.action.parent = 0

# focus will be set on processing the actions to be taken
class CentralControl:

    def __init__(self):
        # initialise node and script files for required subscriptions
        rospy.init_node('central_control')
        self.robot_child = ChildMiro()
        self.robot_parent = ParentMiro()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.controller_pub = rospy.Publisher(
            topic_base_name + '/central_controller', Action, queue_size= 0
        )
        self.action = Action()

        # set variables for the node
        self.time = 0
        # x is needs, y is accumulated needs, 1 is child, 2 is parent
        self.dx1 = -1.0
        self.dy1 = 1.0
        self.dx2 = 0.0
        self.dy2 = 0.0
        self.emotional_distance = 0
        self.epsilonAv = 0.0
        self.epsilonAm = 0.0
        self.a = 1.0
        self.c = 1.0
        self.b = 0.5
        self.dp = 0.0
        self.de = 0.0
        self.h = 0.005

    def central_control(self):
        # action selection variables, do each aciton when the variable is equal to 1
        A_approach = lambda x: np.heaviside(x,0)
        A_explore = lambda x: np.heaviside(-x,0)

        while rospy.is_shutdown():
            self.time += self.h

            # calculate physical distance (max = 1)
            child_position = self.robot_child.odom # Float
            parent_position = self.robot_parent.odom # Float
            self.dp = self.calculate_physical_distance(child_position, parent_position)
            if self.dp > 1:
                self.dp = 1

            # calculate emotional distance (max = 1)
            child_sound = self.robot_child.robot_sound # Boolean
            parent_sound = self.robot_parent.robot_sound # Boolean
            if parent_sound or child_sound:
                if emotional_distance < 100:
                    emotional_distance = 0
            else:
                emotional_distance += (100-emotional_distance)*0.01 
            self.de = emotional_distance/100

            calculated_need_accumulation = [self.dx1, self.dy1, self.dx2, self.dy2]
            # Calculate needs and accumulated needs
            k1 = self.f(self.time, calculated_need_accumulation)
            k2 = self.f(self.time + self.h/2.0, calculated_need_accumulation + self.h*k1/2.0)
            k3 = self.f(self.time + self.h/2.0, calculated_need_accumulation + self.h*k2/2.0)
            k4 = self.f(self.time + self.h, calculated_need_accumulation + self.h*k3)
            calculated_need_accumulation = calculated_need_accumulation + self.h*(k1 + 2*k2 + 2*k3 + k4)/6.0
            # action for child
            if A_approach(calculated_need_accumulation[0], 'k') == 1:
                self.action.child = 0
            elif A_explore(-calculated_need_accumulation[0], 'r') == 1:
                self.action.child = 1
            # action for parent
            if A_approach(calculated_need_accumulation[2], 'k') == 1:
                self.action.parent = 0
            elif A_explore(-calculated_need_accumulation[2], 'r') == 1:
                self.action.parent = 1
            # publish action for child node and parent node to use
            self.controller_pub.pub(self.action)
            # update the needs and accumulated needs
            [self.dx1,self.dy1,self.dx2,self.dy2] = calculated_need_accumulation 
    
    # Calculate distance between two robots using odometry
    def calculate_physical_distance(child_position, parent_position):
        # get from subscriber
        x_c = child_position.pos_x
        x_p = parent_position.pos_x
        y_c = child_position.pos_y
        y_p = parent_position.pos_y
        # calculate distance
        x = x_c - x_p
        y = y_c - y_p
        x_2 = np.square(x)
        y_2 = np.square(y)
        p_d = np.sqrt(x_2 + y_2)
        return p_d

    # Model
    def f(self, t, r):
        x1 = r[0]
        y1 = r[1]
        x2 = r[2]
        y2 = r[3]
        ym = (y1 + y2)/2.0
        xm = (x1 + x2)/2.0
        # Calculate needs and accumulated neds
        dx1 = -self.a*(4.0*self.c*x1**3 - 2.0*x1) - y1 
        dy1 = self.b*x1- self.epsilonAm*(y2 + y1 + self.dp)  - self.epsilonAv*(y2 - y1 - self.de) 
        dx2 = -self.a*(4.0*self.c*x2**3 - 2.0*x2) - y2
        dy2 = self.b*x2  - self.epsilonAm*(y2 + y1 - self.dp)  - self.epsilonAv*(y2 - y1 - self.de)
        return np.array([dx1, dy1, dx2, dy2])
    
if __name__ == '__main__':
    main = CentralControl()
    try:
        main.central_control()
    except rospy.ROSInterruptException:
        pass