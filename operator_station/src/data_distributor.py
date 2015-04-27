#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from yozakura_msgs.msg import YozakuraState, BaseState, ArmState, YozakuraSensorData, HeatSensorData, CO2SensorData
from std_msgs.msg import Float64

DEFAULT_NODE_NAME = 'data_distributor'

# remap-able
DEFAULT_SUB_SENSOR_TOPIC_NAME = 'yozakura_sensor_data'
DEFAULT_PUB_HEAT_TOPIC_NAME = 'heat_data'
DEFAULT_PUB_CO2_TOPIC_NAME = 'co2_data'
DEFAULT_PUB_I_WL_TOPIC_NAME = 'current_wheel_left'
DEFAULT_PUB_I_WR_TOPIC_NAME = 'current_wheel_right'
DEFAULT_PUB_I_FL_TOPIC_NAME = 'current_flipper_left'
DEFAULT_PUB_I_FR_TOPIC_NAME = 'current_flipper_right'
DEFAULT_PUB_V_FR_TOPIC_NAME = 'voltage_base'
DEFAULT_SUB_STATE_TOPIC_NAME = 'yozakura_state'
DEFAULT_PUB_BODY_STATE_TOPIC_NAME = 'body_state'
DEFAULT_PUB_ARM_STATE_TOPIC_NAME = 'arm_state'


class DataDistributor(object):
    def __init__(self):
        self._pub_heat_data = rospy.Publisher(DEFAULT_PUB_HEAT_TOPIC_NAME, HeatSensorData, queue_size=1)
        self._heat_data = HeatSensorData()

        self._pub_co2_data = rospy.Publisher(DEFAULT_PUB_CO2_TOPIC_NAME, CO2SensorData, queue_size=1)
        self._co2_data = CO2SensorData()

        self._pub_i_data_arr = [rospy.Publisher(DEFAULT_PUB_I_WL_TOPIC_NAME, Float64, queue_size=1),
                                rospy.Publisher(DEFAULT_PUB_I_WR_TOPIC_NAME, Float64, queue_size=1),
                                rospy.Publisher(DEFAULT_PUB_I_FL_TOPIC_NAME, Float64, queue_size=1),
                                rospy.Publisher(DEFAULT_PUB_I_FR_TOPIC_NAME, Float64, queue_size=1)]
        # wheel_left, wheel_right, flipper_left, flipper_right
        self._i_data_arr = [Float64()] * len(self._pub_i_data_arr)

        self._pub_v_data = rospy.Publisher(DEFAULT_PUB_V_FR_TOPIC_NAME, Float64, queue_size=1)
        self._v_data = Float64()

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
        self._i_data_arr = [ysensor_data.wheel_left.current if ysensor_data.wheel_left.is_ok else -1.0,
                            ysensor_data.wheel_right.current if ysensor_data.wheel_right.is_ok else -1.0,
                            ysensor_data.flipper_left.current if ysensor_data.flipper_left.is_ok else -1.0,
                            ysensor_data.flipper_right.current if ysensor_data.flipper_right.is_ok else -1.0]
        self._v_data = ysensor_data.wheel_left.voltage if ysensor_data.wheel_left.is_ok else -1.0

    def publish_data(self):
        if not self.is_active:
            self.activate()
        self._pub_base_state.publish(self._base_state)
        self._pub_arm_state.publish(self._arm_state)
        self._pub_heat_data.publish(self._heat_data)
        self._pub_co2_data.publish(self._co2_data)
        for pub_i_data, i_data in zip(self._pub_i_data_arr, self._i_data_arr):
            pub_i_data.publish(i_data)
        self._pub_v_data.publish(self._v_data)


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz

    data_distributor = DataDistributor()
    data_distributor.activate()

    while not rospy.is_shutdown():
        data_distributor.publish_data()
        rate_mgr.sleep()











