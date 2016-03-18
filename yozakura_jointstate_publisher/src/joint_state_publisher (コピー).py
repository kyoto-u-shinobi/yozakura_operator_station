#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
#from yozakura_msgs.msg import ArmState
from std_msgs.msg import Float32MultiArray


class ArmJointStateManager:
    '''
    urdfの表示のためのJointStateを生成してpublishするクラス
    self.basejoint_names, self.jackjoint_names_left, self.jackjoint_names_rightはurdfに依存
    '''

    def __init__(self):

        self.joint_names = ['joint_arm_yaw', 'joint_arm_pitch1', 'joint_arm_pitch2', 'joint_arm_camera']

        self.arm_jointstate = JointState()
        self.arm_jointstate.name = self.joint_names
        self.arm_jointstate.position = [math.radians(0.0)] * len(self.arm_jointstate.name)

    def arm_callback(self, position):
        _position = [0.0, 0.0, 0.0, 0.0]
        _position[0] = math.radians(position.data[0]) * 35 / 120
        _position[1] = math.radians(position.data[1])
        _position[2] = math.radians(position.data[3])
        _position[3] = math.radians(position.data[4])
        self.arm_jointstate.position = _position

    def get_jointstate(self):
        return self.arm_jointstate

    def sub(self):
        rospy.Subscriber("arm_position", Float32MultiArray, self.arm_callback)
'''
if __name__=='__main__':
    rospy.init_node('joint_state_publisher')
    
    yozakura_jointstate = JointState()

    arm_js_mgr = ArmJointStateManager()
    arm_js_mgr.sub()

    try:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rate_mgr = rospy.Rate(10)
        print 'Run!'
        while not rospy.is_shutdown():
            yozakura_jointstate.header.stamp = rospy.Time.now()
            yozakura_jointstate.name = arm_js_mgr.get_jointstate().name
            yozakura_jointstate.position = arm_js_mgr.get_jointstate().position
            yozakura_jointstate.velocity = []
            yozakura_jointstate.effort = []

            pub.publish(yozakura_jointstate)
            rate_mgr.sleep()
    except rospy.ROSInterruptException:
        pass
'''
