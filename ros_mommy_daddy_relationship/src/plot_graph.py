#!/usr/bin/env python3

# for plotting
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import style
matplotlib.use('GTK3Agg')
import cv2
import numpy as np
from matplotlib.widgets import Slider, Button

# for ros
import rospy
from subscribe_controller import ActionMiro
from ros_mommy_daddy_msg.msg import Emotion

class PlotGraph:
    
    def __init__(self):
        # intialise node and required modules for project
        rospy.init_node('child_node')
        rate = rospy.Rate(10)

        # diagram for plotting, the shape of the plot design and data limit for plot
        fig = plt.figure()
        fig.suptitle('MiRo\'s Strange Situation', fontweight="bold", fontsize=18)
        self.path = "/home/aljiro/miro.png"
        self.img = cv2.imread(self.path)

        # miro picture
        self.miro_pic = fig.add_subplot(3,2,1)
        self.miro_pic.axis('off')
        self.miro_pic.imshow(self.img)

        # emotional and physical distance
        self.e_d_p_d = fig.add_subplot(3,2,2)
        self.e_d_p_d.title.set_text("Emotional and Physical Distance")
        self.e_d_p_d.set_ylim(0, 1)

        # child action
        self.child_action = fig.add_subplot(3,2,3)
        self.child_action.title.set_text("Child Action")
        self.child_action.set_xlim(0,1000)
        self.child_action.set_yticks([0, 1], labels = ('Approach', 'Explore'))
        self.child_action.get_xaxis().set_visible(False)

        # parent action
        self.parent_action = fig.add_subplot(3,2,4)
        self.parent_action.title.set_text("Parent Action")
        self.parent_action.set_xlim(0,1000)
        self.parent_action.set_ylim(-0.2, 1.2)
        self.parent_action.get_xaxis().set_visible(False)

        # child need
        self.child_need = fig.add_subplot(3,2,5)
        self.child_need.title.set_text("Child need")
        self.child_need.set_xlim(0,1000)
        self.child_need.get_xaxis().set_visible(False)

        # parent need
        self.parent_need = fig.add_subplot(3,2,6)
        self.parent_need.title.set_text("Parent need")
        self.parent_need.set_xlim(0,1000)
        self.parent_need.get_xaxis().set_visible(False)

        # initialising data for plotting
        self.child_action_data = []
        self.parent_action_data = []
        self.child_need_data = []
        self.parent_need_data = []

        # adjust to make room for slider
        plt.subplots_adjust(bottom=0.25)
        # set epsilon avoidant and slider function
        self.Av = 0
        self.axAv = plt.axes([0.25, 0.15, 0.5, 0.02])
        # slider for avoidant
        self.Av_slider = Slider(
            ax=self.axAv,
            label='Avoidant',
            valmin=0,
            valmax=3,
            valinit=self.Av,
        )

        # set epsilon ambivalent and slider function
        self.Am = 0
        self.axAm = plt.axes([0.25, 0.05, 0.5, 0.02])
        # slider for ambivalent
        self.Am_slider = Slider(
            ax=self.axAm,
            label='Ambivalent',
            valmin=-0.1,
            valmax=2,
            valinit=self.Am,
        )

        # for publishing information given by the slider function
        self.emotion_controller_pub = rospy.Publisher(
            '/emotion_controller', Emotion, queue_size= 0
        )
        self.emotion = Emotion()
        self.emotion.ambivalent = self.Am
        self.emotion.avoidant = self.Av
        
        # Reset to secure button 
        resetax = plt.axes([0.85, 0.04, 0.1, 0.04])
        self.button = Button(resetax, 'Secure', hovercolor='0.975')
        # for subscribing
        self.sub_controller = ActionMiro()
        
        
    
    # reset event
    def reset(self, event):
        self.Am_slider.reset()
        self.Av_slider.reset()

    # The function to be called anytime a slider's value changes
    def update(self, val):
        self.Am = self.Am_slider.val
        self.Av = self.Av_slider.val

    def animate(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # append data
            self.Am_slider.on_changed(self.update)
            self.Av_slider.on_changed(self.update)
            self.button.on_clicked(self.reset)
            self.child_action_data.append(float(self.sub_controller.child))

            # use of fifo to manage data size
            if len(self.child_action_data) > 1000:
                self.child_action_data.pop(0)
            self.parent_action_data.append(float(self.sub_controller.parent))
            if len(self.parent_action_data) > 1000:
                self.parent_action_data.pop(0)
            self.child_need_data.append(float(self.sub_controller.child_need))
            if len(self.child_need_data) > 1000:
                self.child_need_data.pop(0)
            self.parent_need_data.append(float(self.sub_controller.parent_need))
            if len(self.parent_need_data) > 1000:
                self.parent_need_data.pop(0)

            # plot the graph and update them with newly updated data
            self.pe = float(self.sub_controller.emotional_distance)
            self.pd = float(self.sub_controller.physical_distance)
            data = {"Emotional Distance": self.pe, "Physical Distance": self.pd}
            distances = list(data.keys())
            values = list(data.values())
            self.e_d_p_d.clear()
            self.e_d_p_d.title.set_text("Emotional and Physical Distance")
            self.e_d_p_d.set_ylim(0, 1)
            barlist = self.e_d_p_d.bar(distances, values, width = 0.4)
            barlist[0].set_color('maroon')
            barlist[1].set_color('blue')
            self.parent_action.clear()
            self.parent_action.set_xlim(0,1000)
            self.parent_action.set_ylim(-0.2, 1.2)
            self.parent_action.title.set_text("Parent Action")
            self.parent_action.plot(self.parent_action_data, color = "brown", linewidth = 2)
            self.parent_action.get_xaxis().set_visible(False)
            self.child_action.clear()
            self.child_action.set_xlim(0,1000)
            self.child_action.set_ylim(-0.2, 1.2)
            self.child_action.title.set_text("Child Action")
            self.child_action.plot(self.child_action_data, color = "red", linewidth = 2)
            self.child_action.get_xaxis().set_visible(False)
            self.child_need.clear()
            self.child_need.set_xlim(0,1000)
            self.child_need.set_ylim(-2,2)
            self.child_need.title.set_text("Child need")
            self.child_need.plot(self.child_need_data, color = "red", linewidth = 2)
            self.child_need.get_xaxis().set_visible(False)
            self.parent_need.clear()
            self.parent_need.set_xlim(0,1000)
            self.parent_need.set_ylim(-2,2)
            self.parent_need.title.set_text("Parent need")
            self.parent_need.plot(self.parent_need_data, color = "brown", linewidth = 2)
            self.parent_need.get_xaxis().set_visible(False)

            self.child_action.set_yticks([0, 1], labels = ('Approach', 'Explore'))
            self.parent_action.set_yticks([0, 1], labels = ('Approach', 'Explore'))
            plt.pause(0.05)
            self.emotion.ambivalent = self.Am
            self.emotion.avoidant = self.Av
            self.emotion_controller_pub.publish(self.emotion)

            rate.sleep()


if __name__ == '__main__':
    main = PlotGraph()
    try:
        main.animate()
    except rospy.ROSInterruptException:
        pass