#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32


class BodyJointStateManager:
    def __init__(self):
        self.bodyjoint_names = ['joint_body_front_pitch', 'joint_body_front_roll',
                                'joint_body_pitch', 'joint_body_roll']

        self.flipperjoint_names = ['joint_flipper_left', 'joint_flipper_right']

        self.wheeljoint_names = ['joint_starwheel_front', 'joint_starwheel_back']

        self.base_jointstate = {}
        for name in self.bodyjoint_names + self.flipperjoint_names + self.wheeljoint_names:
            self.base_jointstate[name] = np.radians(0.0)

    def __bodyfront_pitch_callback(self, float_msg):
        self.base_jointstate[self.bodyjoint_names[0]] = float_msg.data

    def __bodyfront_roll_callback(self, float_msg):
        self.base_jointstate[self.bodyjoint_names[1]] = float_msg.data

    def __body_back_pitch_callback(self, float_msg):
        self.base_jointstate[self.bodyjoint_names[2]] = float_msg.data \
                                                        - self.base_jointstate[self.bodyjoint_names[0]]

    def __body_back_roll_callback(self, float_msg):
        self.base_jointstate[self.bodyjoint_names[3]] = float_msg.data \
                                                        - self.base_jointstate[self.bodyjoint_names[1]]

    def __flipper_left_callback(self, float_msg):
        self.base_jointstate[self.flipperjoint_names[0]] = float_msg.data

    def __flipper_right_callback(self, float_msg):
        self.base_jointstate[self.flipperjoint_names[1]] = float_msg.data

    def __starwheel_front_callback(self, float_msg):
        self.base_jointstate[self.wheeljoint_names[0]] = float_msg.data

    def __starwheel_back_callback(self, float_msg):
        self.base_jointstate[self.wheeljoint_names[1]] = float_msg.data


    def get_jointstate_name(self):
        return self.base_jointstate.keys()

    def get_jointstate_position(self):
        return self.base_jointstate.values()

    def run(self):
        rospy.Subscriber("body_front_pitch_deg", Float32, self.__bodyfront_pitch_callback)
        rospy.Subscriber("body_front_roll_deg", Float32, self.__bodyfront_roll_callback)
        rospy.Subscriber("body_back_pitch_deg", Float32, self.__body_back_pitch_callback)
        rospy.Subscriber("body_back_roll_deg", Float32, self.__body_back_roll_callback)
        rospy.Subscriber("flipper_left_deg", Float32, self.__flipper_left_callback)
        rospy.Subscriber("flipper_right_deg", Float32, self.__flipper_right_callback)
        rospy.Subscriber("starwheel_front_deg", Float32, self.__starwheel_front_callback)
        rospy.Subscriber("starwheel_back_deg", Float32, self.__starwheel_back_callback)


