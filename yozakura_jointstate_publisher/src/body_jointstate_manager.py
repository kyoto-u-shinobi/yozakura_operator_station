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
        bodyjoint_names = ['joint_body_front_pitch', 'joint_body_front_roll',
                           'joint_body_back_pitch', 'joint_body_back_roll']

        flipperjoint_names = ['joint_flipper_left', 'joint_flipper_right']

        wheeljoint_names = ['joint_starwheel_front', 'joint_starwheel_back']

        self.base_jointstate = JointState()
        self.base_jointstate.name = bodyjoint_names + flipperjoint_names + wheeljoint_names
        # the return of np.rad2deg is numpy.array. map can convert numpy.array to list
        self.base_jointstate.position = map(None, np.deg2rad([0.0] * len(self.base_jointstate.name)))

    def __base_callback(self, base_state):
        self.base_jointstate.position = map(None, np.deg2rad([base_state.body_front.pitch,
                                                                  base_state.body_front.roll,
                                                                  base_state.body_back.pitch - base_state.body_front.pitch,
                                                                  base_state.body_back.roll - base_state.body_front.roll,
                                                                  base_state.flipper_left.angle,
                                                                  base_state.flipper_right.angle,
                                                                  base_state.wheel_left.rotation_angle,
                                                                  base_state.wheel_right.rotation_angle]))

    def get_jointstate(self):
        return self.base_jointstate

    def run(self):
        rospy.Subscriber("body_state", BaseState, self.__base_callback)


