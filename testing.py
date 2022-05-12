#!/usr/bin/env python

import rospy
import numpy as np
from node_wag_tail import NodeWagTail
from node_make_audio import NodeMakeAudio
from node_detect_audio import NodeDetectAudio

class Testing:
    def __init__(self):
        # initialise node and required modules for the project
        rospy.init_node('testing')
        self.robot_wag = NodeWagTail()
        self.robot_make_audio = NodeMakeAudio()
        self.robot_detect_sound = NodeDetectAudio()
        
        # can be removed
        self.termination = False
        self.time = 0

    def wag_hard(self):
        while not self.termination:
            self.time += 1
            #move back and forth
            tail_direction = np.absolute(np.sin(self.time))
            self.robot_wag.set_wag_cmd(wag= tail_direction)
            self.robot_wag.pub_wag()
            #self.robot_make_audio.set_sound_cmd(freq=1000, volume=50, duration=255)
            #self.robot_make_audio.pub_sound()
            print('wag bish')


    def make_sound(self):
        self.robot_make_audio.produce_sound(freq=500, volume=255, duration=63)
        #self.robot_make_audio.pub_sound()

    def detect_sound(self):
        self.robot_detect_sound.detect_sound_cmd(freq=500, volume=255, duration=63)
        self.robot_detect_sound.pub_detect_sound()

        

if __name__ == '__main__':
    main = Testing()
    main.wag_hard()
    main.make_sound()
