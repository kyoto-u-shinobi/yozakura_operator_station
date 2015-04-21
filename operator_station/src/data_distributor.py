#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from yozakura_msgs.msg import YozakuraSensorData, HeatSensorData, CO2SensorData
from std_msgs.msg import Float64

VOLT_DATA_NUM = 5
DEFAULT_NODE_NAME = 'data_distributor'

# remap-able
DEFAULT_SUB_TOPIC_NAME = 'yozakura_sensor_data'
DEFAULT_PUB_HEAT_TOPIC_NAME = 'heat_data'
DEFAULT_PUB_CO2_TOPIC_NAME = 'co2_data'
DEFAULT_PUB_VOLT_WL_TOPIC_NAME = 'volt_wheel_left'
DEFAULT_PUB_VOLT_WR_TOPIC_NAME = 'volt_wheel_right'
DEFAULT_PUB_VOLT_FL_TOPIC_NAME = 'volt_flipper_left'
DEFAULT_PUB_VOLT_FR_TOPIC_NAME = 'volt_flipper_right'
DEFAULT_PUB_VOLT_BAT_TOPIC_NAME = 'volt_battery'


class DataDistributor(object):
    def __init__(self):
        self.heat_data = HeatSensorData()
        self.pub_heat_data = rospy.Publisher(DEFAULT_PUB_HEAT_TOPIC_NAME, HeatSensorData, queue_size=1)

        self.co2_data = CO2SensorData()
        self.pub_co2_data = rospy.Publisher(DEFAULT_PUB_CO2_TOPIC_NAME, CO2SensorData, queue_size=1)

        # wheel_left, wheel_right, flipper_left, flipper_right, battery
        self.volt_data_arr = [Float64()] * VOLT_DATA_NUM
        self.pub_volt_data_arr = [rospy.Publisher(DEFAULT_PUB_VOLT_WL_TOPIC_NAME, Float64, queue_size=1),
                                  rospy.Publisher(DEFAULT_PUB_VOLT_WR_TOPIC_NAME, Float64, queue_size=1),
                                  rospy.Publisher(DEFAULT_PUB_VOLT_FL_TOPIC_NAME, Float64, queue_size=1),
                                  rospy.Publisher(DEFAULT_PUB_VOLT_FR_TOPIC_NAME, Float64, queue_size=1),
                                  rospy.Publisher(DEFAULT_PUB_VOLT_BAT_TOPIC_NAME, Float64, queue_size=1)]

        self.is_active = False

    def activate(self):
        rospy.Subscriber(DEFAULT_SUB_TOPIC_NAME, YozakuraSensorData, self.ysensor_data_callback)
        self.is_active = True

    def ysensor_data_callback(self, ysensor_data):
        self.heat_data = ysensor_data.heat
        self.co2_data = ysensor_data.co2
        self.volt_data_arr = [ysensor_data.wheel_left.voltage if ysensor_data.wheel_left.is_ok else -1.0,
                              ysensor_data.wheel_right.voltage if ysensor_data.wheel_right.is_ok else -1.0,
                              ysensor_data.flipper_left.voltage if ysensor_data.flipper_left.is_ok else -1.0,
                              ysensor_data.flipper_right.voltage if ysensor_data.flipper_right.is_ok else -1.0,
                              ysensor_data.battery.voltage if ysensor_data.battery.is_ok else -1.0]

    def publish_data(self):
        if not self.is_active:
            self.activate()

        self.pub_heat_data.publish(self.heat_data)
        self.pub_co2_data.publish(self.co2_data)
        for i in range(VOLT_DATA_NUM):
            self.pub_volt_data_arr[i].publish(self.volt_data_arr[i])


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz

    data_distributor = DataDistributor()
    data_distributor.activate()

    while not rospy.is_shutdown():
        data_distributor.publish_data()
        rate_mgr.sleep()











