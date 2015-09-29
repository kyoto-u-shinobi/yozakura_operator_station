#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from yozakura_msgs.msg import BaseState


class BodyJointStateManager:
    '''
    urdfの表示のためのJointStateを生成してpublishするクラス
    self.bodyjoint_names, self.flipperjoint_names, self.wheeljoint_namesはurdfで定義
    '''

    def __init__(self):
        bodyjoint_names = ['joint_body_front_pitch', 'joint_body_front_roll',
                           'joint_body_back_pitch', 'joint_body_back_roll']

        flipperjoint_names = ['joint_flipper_left', 'joint_flipper_right']

        wheeljoint_names = ['joint_starwheel_front', 'joint_starwheel_back']

        self.base_jointstate = JointState()
        self.base_jointstate.name = bodyjoint_names + flipperjoint_names + wheeljoint_names
        # the return of np.rad2deg is numpy.array. map can convert numpy.array to list
        self.base_jointstate.position = [math.radians(0.0)] * len(self.base_jointstate.name)

    def _base_callback(self, base_state):
        # rvizの表示に使っているdaeファイルの座標系的にbodyのpitchとflipperの向きが逆
        _positions = [-base_state.body_front.pitch,
                      base_state.body_front.roll,
                      -(base_state.body_back.pitch - base_state.body_front.pitch),
                      base_state.body_back.roll - base_state.body_front.roll,
                      -base_state.flipper_left.angle,
                      -base_state.flipper_right.angle,
                      -base_state.wheel_left.rotation_angle,
                      -base_state.wheel_right.rotation_angle]

        self.base_jointstate.position = [math.radians(pos) for pos in _positions]

    def get_jointstate(self):
        return self.base_jointstate

    def run(self):
        rospy.Subscriber("body_state", BaseState, self._base_callback)


