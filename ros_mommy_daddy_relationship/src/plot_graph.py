#!/usr/bin/env python3

# for plotting
import matplotlib.pyplot as plt
from matplotlib import style
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

        # for plot
        fig = plt.figure()
        self.child_action = fig.add_subplot(3,2,1)
        self.child_action.title.set_text("Child Action")
        self.parent_action = fig.add_subplot(3,2,2)
        self.parent_action.title.set_text("Parent Action")
        self.pe = fig.add_subplot(3,2,3)
        self.pe.title.set_text("Emotional distance")
        self.pd = fig.add_subplot(3,2,4)
        self.pd.title.set_text("Physical distance")
        self.child_need = fig.add_subplot(3,2,5)
        self.child_need.title.set_text("Child need")
        self.parent_need = fig.add_subplot(3,2,6)
        self.parent_need.title.set_text("Parent need")
        self.child_action_data = []
        self.parent_action_data = []
        self.pe_data = []
        self.pd_data = []
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
            valmin=0,
            valmax=3,
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
            self.parent_action_data.append(float(self.sub_controller.parent))
            self.pe_data.append(float(self.sub_controller.emotional_distance))
            self.pd_data.append(float(self.sub_controller.physical_distance))
            self.child_need_data.append(float(self.sub_controller.child_need))
            self.parent_need_data.append(float(self.sub_controller.parent_need))

            self.child_action.plot(self.child_action_data, color = "red")
            self.parent_action.plot(self.parent_action_data, color = "red")
            self.pe.plot(self.pe_data, color = "red")
            self.pd.plot(self.pd_data, color = "red")
            self.child_need.plot(self.child_need_data, color = "red")
            self.parent_need.plot(self.parent_need_data, color = "red")
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