#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from yozakura_msgs.msg import YozakuraState, BaseState, ArmState, YozakuraSensorData, HeatSensorData, CO2SensorData
from std_msgs.msg import Float32


# remap-able
DEFAULT_NODE_NAME = 'data_distributor'
DEFAULT_SUB_SENSOR_TOPIC_NAME = 'yozakura_sensor_data'
DEFAULT_PUB_HEAT_TOPIC_NAME = 'heat_data'
DEFAULT_PUB_CO2_TOPIC_NAME = 'co2_data'
DEFAULT_PUB_I_TOPIC_NAME = 'current_data'
DEFAULT_PUB_V_TOPIC_NAME = 'voltage_data'
DEFAULT_SUB_STATE_TOPIC_NAME = 'yozakura_state'
DEFAULT_PUB_BODY_STATE_TOPIC_NAME = 'body_state'
DEFAULT_PUB_ARM_STATE_TOPIC_NAME = 'arm_state'


class DataDistributor(object):
    def __init__(self):
        self._pub_heat_data = rospy.Publisher(DEFAULT_PUB_HEAT_TOPIC_NAME, HeatSensorData, queue_size=1)
        self._heat_data = HeatSensorData()

        self._pub_co2_data = rospy.Publisher(DEFAULT_PUB_CO2_TOPIC_NAME, CO2SensorData, queue_size=1)
        self._co2_data = CO2SensorData()

        self._pub_i_data = rospy.Publisher(DEFAULT_PUB_I_TOPIC_NAME, Float32, queue_size=1)
        self._i_data = Float32()

        self._pub_v_data = rospy.Publisher(DEFAULT_PUB_V_TOPIC_NAME, Float32, queue_size=1)
        self._v_data = Float32()

        self._pub_base_state = rospy.Publisher(DEFAULT_PUB_BODY_STATE_TOPIC_NAME, BaseState, queue_size=1)
        self._base_state = BaseState()

        self._pub_arm_state = rospy.Publisher(DEFAULT_PUB_ARM_STATE_TOPIC_NAME, ArmState, queue_size=1)
        self._arm_state = ArmState()

        self.is_active = False

    def activate(self):
        rospy.Subscriber(DEFAULT_SUB_SENSOR_TOPIC_NAME, YozakuraSensorData, self.ysensor_data_callback)
        rospy.Subscriber(DEFAULT_SUB_STATE_TOPIC_NAME, YozakuraState, self.ystate_callback)

        self.is_active = True

    def ystate_callback(self, ystate):
        self._base_state = ystate.base
        self._arm_state = ystate.arm

    def ysensor_data_callback(self, ysensor_data):
        self._heat_data = ysensor_data.heat
        self._co2_data = ysensor_data.co2
        self._v_data = ysensor_data.wheel_left.voltage if ysensor_data.wheel_left.is_ok else -1.0
        self._i_data = ysensor_data.wheel_left.current + \
                       ysensor_data.wheel_right.current + \
                       ysensor_data.flipper_left.current + \
                       ysensor_data.flipper_right.current



    def publish_data(self):
        if not self.is_active:
            self.activate()
        self._pub_base_state.publish(self._base_state)
        self._pub_arm_state.publish(self._arm_state)
        self._pub_heat_data.publish(self._heat_data)
        self._pub_co2_data.publish(self._co2_data)
        self._pub_v_data.publish(self._v_data)
        self._pub_i_data.publish(self._i_data)


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz

    data_distributor = DataDistributor()
    data_distributor.activate()

    while not rospy.is_shutdown():
        data_distributor.publish_data()
        rate_mgr.sleep()











