#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class MyArmJointStatePublisher:
    def __init__(self):
        self.joint_state_msg = JointState()

        self.base_yaw_angle = np.radians(0.0)
        self.base_pitch_angle = np.radians(0.0)
        self.jack_triangle_base_angle = np.radians(0.0)
        self.link_length = 0.200

        self.basejoint_names = ['joint_arm_yaw', 'joint_arm_pitch']

        self.jackjoint_names_left = ['virtualJoint_armLinear_base_left',
                                     'joint_armLinear_left_A_1', 'joint_armLinear_left_B_1',
                                     'joint_armLinear_left_A_2', 'joint_armLinear_left_B_2',
                                     'joint_armLinear_left_A_3', 'joint_armLinear_left_B_3',
                                     'joint_armLinear_left_A_4', 'joint_armLinear_left_B_4']

        self.jackjoint_names_right = ['virtualJoint_armLinear_base_right',
                                      'joint_armLinear_right_A_1', 'joint_armLinear_right_B_1',
                                      'joint_armLinear_right_A_2', 'joint_armLinear_right_B_2',
                                      'joint_armLinear_right_A_3', 'joint_armLinear_right_B_3',
                                      'joint_armLinear_right_A_4', 'joint_armLinear_right_B_4']

    def get_pos_jacks_arr(self, jackjoint_names, sig):
        ret_arr = range(len(jackjoint_names))
        for idx, name in enumerate(jackjoint_names):
            if idx == 0:  # the 1st joint is slider
                ret_arr[idx] = sig * (self.link_length / 2.0) * (1.0 - np.cos(self.jack_triangle_base_angle / 2.0))
            elif idx == 1:  # the 2nd joint is half joint
                ret_arr[idx] = -sig * self.jack_triangle_base_angle / 2.0
            elif idx % 2 == 1:
                ret_arr[idx] = -sig * self.jack_triangle_base_angle
            elif idx % 2 == 0:
                ret_arr[idx] = sig * self.jack_triangle_base_angle
            else:
                ret_arr[idx] = 0.0
        return ret_arr

    def get_pos_jacks_arr_left(self):
        return self.get_pos_jacks_arr(self.jackjoint_names_left, 1.0)

    def get_pos_jacks_arr_right(self):
        return self.get_pos_jacks_arr(self.jackjoint_names_right, -1.0)

    def armjack_callback(self, top_angle):
        self.jack_triangle_base_angle = np.radians((360.0 - 2.0 * float(top_angle.data)) / 2.0)

    def armbase_yaw_callback(self, yaw_angle):
        self.base_yaw_angle = np.radians(yaw_angle.data)

    def armbase_pitch_callback(self, pitch_angle):
        self.base_pitch_angle = np.radians(pitch_angle.data)

    def main(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("armjack_triangle_top_angle_deg", Float32, self.armjack_callback)
        rospy.Subscriber("armbase_yaw_angle_deg", Float32, self.armbase_yaw_callback)
        rospy.Subscriber("armbase_pitch_angle_deg", Float32, self.armbase_pitch_callback)

        pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            print 'RUN'
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_msg.name = self.basejoint_names + self.jackjoint_names_left + self.jackjoint_names_right
            self.joint_state_msg.position = [self.base_yaw_angle, self.base_pitch_angle] + self.get_pos_jacks_arr_left() + self.get_pos_jacks_arr_right()
            self.joint_state_msg.velocity = []
            self.joint_state_msg.effort = []
            pub.publish(self.joint_state_msg)
            r.sleep()  # this has rospy.spin()


if __name__ == '__main__':
    my_js_pub = MyArmJointStatePublisher()
    try:
        my_js_pub.main()
    except rospy.ROSInterruptException:
        pass
