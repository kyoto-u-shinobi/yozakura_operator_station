#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32


class ArmJointStateManager:
    def __init__(self):
        self.base_yaw_angle = np.radians(0.0)
        self.base_pitch_angle = np.radians(0.0)
        self.jack_triangle_base_angle = np.radians(0.0)
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

    def __get_pos_jacks_arr(self, jackjoint_names, sig):
        ret_arr = range(len(jackjoint_names))
        for idx, name in enumerate(jackjoint_names):
            if idx == 0:  # the 1st joint is slider
                ret_arr[idx] = sig * (self.__link_length / 2.0) * (1.0 - np.cos(self.jack_triangle_base_angle / 2.0))
            elif idx == 1:  # the 2nd joint is half joint
                ret_arr[idx] = -sig * self.jack_triangle_base_angle / 2.0
            elif idx % 2 == 1:
                ret_arr[idx] = -sig * self.jack_triangle_base_angle
            elif idx % 2 == 0:
                ret_arr[idx] = sig * self.jack_triangle_base_angle
            else:
                ret_arr[idx] = 0.0
        return ret_arr

    def __get_pos_jacks_arr_left(self):
        return self.__get_pos_jacks_arr(self.jackjoint_names_left, 1.0)

    def __get_pos_jacks_arr_right(self):
        return self.__get_pos_jacks_arr(self.jackjoint_names_right, -1.0)

    def __armjack_callback(self, top_angle):
        self.jack_triangle_base_angle = np.radians((360.0 - 2.0 * float(top_angle.data)) / 2.0)

    def __armbase_yaw_callback(self, yaw_angle):
        self.base_yaw_angle = np.radians(yaw_angle.data)

    def __armbase_pitch_callback(self, pitch_angle):
        self.base_pitch_angle = np.radians(pitch_angle.data)

    def get_jointstate_name(self):
        return self.basejoint_names + self.jackjoint_names_left + self.jackjoint_names_right

    def get_jointstate_position(self):
        return [self.base_yaw_angle,
                self.base_pitch_angle] + self.__get_pos_jacks_arr_left() + self.__get_pos_jacks_arr_right()

    def run(self):
        rospy.Subscriber("armjack_triangle_topangle_deg", Float32, self.__armjack_callback)
        rospy.Subscriber("armbase_yaw_deg", Float32, self.__armbase_yaw_callback)
        rospy.Subscriber("armbase_pitch_deg", Float32, self.__armbase_pitch_callback)

