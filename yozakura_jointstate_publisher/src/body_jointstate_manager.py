#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from yozakura_msgs.msg import BaseState


class BodyJointStateManager:
    '''
    urdfの表示のためのJointStateを生成してpublishするクラス
    self.bodyjoint_names, self.flipperjoint_names, self.wheeljoint_namesはurdfに依存
    '''

    def __init__(self):
        self.bodyjoint_names = ['joint_body_front_pitch', 'joint_body_front_roll',
                                'joint_body_back_pitch', 'joint_body_back_roll']

        self.flipperjoint_names = ['joint_flipper_left', 'joint_flipper_right']

        self.wheeljoint_names = ['joint_starwheel_front', 'joint_starwheel_back']

        self.base_jointstate = JointState()
        self.base_jointstate.name = self.bodyjoint_names + self.flipperjoint_names + self.wheeljoint_names
        self.base_jointstate.position = [np.radians(0.0)] * len(self.base_jointstate.name)

    def __base_callback(self, base_state):
        if base_state.is_ok:
            self.base_jointstate.position = []
            self.base_jointstate.position.append(np.radians(base_state.body_front.pitch))
            self.base_jointstate.position.append(np.radians(base_state.body_front.roll))
            self.base_jointstate.position.append(np.radians(base_state.body_back.pitch))
            self.base_jointstate.position.append(np.radians(base_state.body_back.roll))
            self.base_jointstate.position.append(np.radians(base_state.flipper_left))
            self.base_jointstate.position.append(np.radians(base_state.flipper_right))
            self.base_jointstate.position.append(np.radians(base_state.wheel_left))
            self.base_jointstate.position.append(np.radians(base_state.wheel_right))

    def get_jointstate(self):
        return self.base_jointstate

    def run(self):
        rospy.Subscriber("body_state", BaseState, self.__base_callback)


