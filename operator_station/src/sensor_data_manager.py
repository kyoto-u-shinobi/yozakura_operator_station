#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
from yozakura_msgs.msg import YozakuraSensorData, YozakuraState


# remap-able names
DEFAULT_STATE_TOPIC_NAME = "yozakura_state"
DEFAULT_SENSORDATA_TOPIC_NAME = "yozakura_sensor_data"


class SensorDataManager(object):
    def __init__(self):
        self._ystate = YozakuraState()
        self._pub_ystate = rospy.Publisher(DEFAULT_STATE_TOPIC_NAME, YozakuraState, queue_size=10)

        self._ysensor_data = YozakuraSensorData()
        self._pub_ysensor_data = rospy.Publisher(DEFAULT_SENSORDATA_TOPIC_NAME, YozakuraSensorData, queue_size=10)

        # ポテンショ生データ→角度[deg]: [0, 1]→[0, 3600]
        # ギア比　ポテンショ[deg]:フリッパー[deg] = 50:16
        self.flipper_raw2deg = 3600.0 * (16.0 / 50.0)
        self._lflipper_center_raw_data = rospy.get_param('~lflipper_center_raw_data', 0.307)
        self._rflipper_center_raw_data = rospy.get_param('~rflipper_center_raw_data', 0.608)

    def publish_data(self):
        self._pub_ystate.publish(self._ystate)
        self._pub_ysensor_data.publish(self._ysensor_data)

    def _convert_data(self, new_raw_data, current_data, scale, offset):
        # print(new_raw_data, current_data)
        if new_raw_data is None:
            return False, current_data
        else:
            return True, scale * (new_raw_data + offset)

    def _set_flipper_angles(self, ystate_flipper, flipper_angles, scale, offset):
        ystate_flipper.is_ok, ystate_flipper.angle = self._convert_data(flipper_angles,
                                                                        ystate_flipper.angle,
                                                                        scale, offset)

    def _set_current_sensor_data(self, ysensordata_current,
                                 current_data,
                                 current_scale, current_offset,
                                 voltage_scale, voltage_offset):

        flag_current, ysensordata_current.current = self._convert_data(current_data[0],
                                                                       ysensordata_current.current,
                                                                       current_scale, current_offset)

        flag_voltage, ysensordata_current.voltage = self._convert_data(current_data[2],
                                                                       ysensordata_current.voltage,
                                                                       voltage_scale, voltage_offset)

        ysensordata_current.is_ok = flag_current and flag_voltage

    def _set_pose_sensor(self, ystate_body, pose_data,
                         roll_scale, roll_offset,
                         pitch_scale, pitch_offset,
                         yaw_scale, yaw_offset):

        flag_roll, ystate_body.roll = self._convert_data(pose_data[0],
                                                         ystate_body.roll,
                                                         roll_scale, roll_offset)

        flag_pitch, ystate_body.pitch = self._convert_data(pose_data[1],
                                                           ystate_body.pitch,
                                                           pitch_scale, pitch_offset)

        flag_yaw, ystate_body.yaw = self._convert_data(pose_data[2],
                                                       ystate_body.yaw,
                                                       yaw_scale, yaw_offset)

        ystate_body.is_ok = flag_roll and flag_pitch and flag_yaw


    def set_data(self, flipper_angles, current_sensor_data, imu_sensor_data):
        # print(flipper_angles, current_sensor_data, imu_sensor_data)

        self._set_flipper_angles(self._ystate.base.flipper_left,
                                 flipper_angles[0],
                                 self.flipper_raw2deg, -self._lflipper_center_raw_data)
        self._set_flipper_angles(self._ystate.base.flipper_right,
                                 flipper_angles[1],
                                 -self.flipper_raw2deg, -self._rflipper_center_raw_data)

        lwheel, rwheel, lflip, rflip, battery = current_sensor_data
        self._set_current_sensor_data(self._ysensor_data.wheel_left,
                                      lwheel,
                                      1.0, 0.0,
                                      1.0, 0.0)
        self._set_current_sensor_data(self._ysensor_data.wheel_right,
                                      rwheel,
                                      1.0, 0.0,
                                      1.0, 0.0)
        self._set_current_sensor_data(self._ysensor_data.flipper_left,
                                      lflip,
                                      1.0, 0.0,
                                      1.0, 0.0)
        self._set_current_sensor_data(self._ysensor_data.flipper_right,
                                      rflip,
                                      1.0, 0.0,
                                      1.0, 0.0)
        self._set_current_sensor_data(self._ysensor_data.battery,
                                      battery,
                                      1.0, 0.0,
                                      1.0, 0.0)

        front, back = imu_sensor_data
        # もしNoneじゃなかったらdegにする．そうじゃないならNone
        front = [np.rad2deg(data) if data is not None else None for data in front]
        back = [np.rad2deg(data) if data is not None else None for data in back]
        self._set_pose_sensor(self._ystate.base.body_front,
                              front,
                              1.0, 0.0,
                              1.0, 0.0,
                              1.0, 0.0)
        self._set_pose_sensor(self._ystate.base.body_back,
                              back,
                              1.0, 0.0,
                              1.0, 0.0,
                              1.0, 0.0)


if __name__ == "__main__":
    rospy.init_node('sensor_data_manager', anonymous=True)
    sensor_data_mgr = SensorDataManager()

    rate_mgr = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown():
        flipper_angle = [0.0, 0.0]  # left, right
        lwheel_current = [0.0, 0.0, 0.0]  # current, power, voltage
        rwheel_current = [0.0, 0.0, 0.0]
        lflipper_current = [0.0, 0.0, 0.0]
        rflipper_current = [0.0, 0.0, 0.0]
        battery_current = [0.0, 0.0, 0.0]
        front_pose = [0.0, 0.0, 0.0]  # roll, pitch, yaw [rad]
        back_pose = [0.0, 0.0, 0.0]

        # set data
        sensor_data_mgr.set_data(flipper_angle,
                                 [lwheel_current, rwheel_current, lflipper_current, rflipper_current, battery_current],
                                 [front_pose, back_pose])

        # this publishes data that is aligned and converted to ros-style data
        sensor_data_mgr.publish_data()
        rate_mgr.sleep()







