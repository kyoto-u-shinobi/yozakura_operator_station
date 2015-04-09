#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from yozakura_msgs.msg import ArmState


class ArmJointStateManager:
    '''
    urdfの表示のためのJointStateを生成してpublishするクラス
    self.basejoint_names, self.jackjoint_names_left, self.jackjoint_names_rightはurdfに依存
    '''

    def __init__(self):
        self.__link_length = 0.200

        self.basejoint_names = ['joint_arm_yaw', 'joint_arm_pitch']

        self.jackjoint_names_left = ['virtualJoint_armLinear_base_left',
                                     'joint_armLinear_left_A_1', 'joint_armLinear_left_B_1',
                                     'joint_armLinear_left_A_2', 'joint_armLinear_left_B_2',
                                     'joint_armLinear_left_A_3', 'joint_armLinear_left_B_3']

        self.jackjoint_names_right = ['virtualJoint_armLinear_base_right',
                                      'joint_armLinear_right_A_1', 'joint_armLinear_right_B_1',
                                      'joint_armLinear_right_A_2', 'joint_armLinear_right_B_2',
                                      'joint_armLinear_right_A_3', 'joint_armLinear_right_B_3']

        self.arm_jointstate = JointState()
        self.arm_jointstate.name = self.basejoint_names + self.jackjoint_names_left + self.jackjoint_names_right
        self.arm_jointstate.position = [np.deg2rad(0.0)] * len(self.arm_jointstate.name)

    def __get_pos_jacks_arr(self, jackjoint_names, jack_base_angle, is_left):
        '''
        urdfではアームの1リンクごとの姿勢を指定してやる必要があるので，
        ここでjack_triangle_base_angleを使って，そいつらを計算している．
        この関数の説明むずかしい．．．
        :param jackjoint_names: returnの配列に対応
        :param jack_base_angle: アームの◇になってるところの>の角度
        :param is_left: 左右反転用．＜ならTrue，＞ならFalse
        :return: 1リンクごとの角度の配列
        '''
        ret_arr = range(len(jackjoint_names))
        sig = 1.0 if is_left else -1.0
        for idx, name in enumerate(jackjoint_names):
            if idx == 0:  # the 1st joint is slider
                ret_arr[idx] = sig * (self.__link_length / 2.0) * (1.0 - np.cos(jack_base_angle / 2.0))
            elif idx == 1:  # the 2nd joint is half joint
                ret_arr[idx] = -sig * jack_base_angle / 2.0
            elif idx % 2 == 1:
                ret_arr[idx] = -sig * jack_base_angle
            elif idx % 2 == 0:
                ret_arr[idx] = sig * jack_base_angle
            else:
                ret_arr[idx] = 0.0
        return ret_arr

    def __get_pos_jacks_arr_left(self, jack_base_angle):
        return self.__get_pos_jacks_arr(self.jackjoint_names_left, jack_base_angle, True)

    def __get_pos_jacks_arr_right(self, jack_base_angle):
        return self.__get_pos_jacks_arr(self.jackjoint_names_right, jack_base_angle, False)

    def __arm_callback(self, arm_state):
        if arm_state.is_ok:
            jack_base_angle = np.deg2rad((360.0 - 2.0 * float(arm_state.top_angle)) / 2.0)
            # np.deg2rad returns np.array. map can convert np.array to list
            self.arm_jointstate.position = map(None, np.deg2rad([arm_state.yaw, arm_state.pitch])) \
                                           + self.__get_pos_jacks_arr_left(jack_base_angle) \
                                           + self.__get_pos_jacks_arr_right(jack_base_angle)

    def get_jointstate(self):
        return self.arm_jointstate

    def run(self):
        rospy.Subscriber("arm_state", ArmState, self.__arm_callback)

