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
        self._ysensor_data.heat.data = [0.0] * 32
        # ポテンショ生データ→角度[deg]: [0, 1]→[0, 3600]
        # ギア比　ポテンショ[deg]:フリッパー[deg] = 50:16
        self._flipper_raw2deg = 3600.0 * (16.0 / 50.0)
        self._lflipper_center_raw_data = rospy.get_param('~lflipper_center_raw_data', 0.33780)
        self._rflipper_center_raw_data = rospy.get_param('~rflipper_center_raw_data', 0.61080)

        # DXの角度を実際の角度に変換する
        # linearはDXの角度からじゃばらアームリンクの作る菱型の広い方の角度に変換しないといけない
        self._arm_linear_dxdeg2armdeg = (20.0 / 100.0)  # (実験的に作る)
        self._arm_yaw_dxdeg2armdeg = -(1.0 / 5.0)
        self._arm_pitch_dxdeg2armdeg = -(24.0 / 50.0)
        # DXの初期姿勢のときの角度
        self._arm_linear_center_dxdeg = 570.0
        self._arm_yaw_center_dxdeg = 0.0
        self._arm_pitch_center_dxdeg = 334.0

    def publish_data(self):
        self._pub_ystate.publish(self._ystate)
        self._pub_ysensor_data.publish(self._ysensor_data)

    @staticmethod
    def _convert_data(new_raw_data, current_data, scale, offset):
        if isinstance(new_raw_data, list):
            if new_raw_data.count(None) is len(new_raw_data):
                return False, current_data
            else:
                return True, [scale * (data + offset) if data is not None else None for data in new_raw_data]
        else:
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
        flag_voltage, ysensordata_current.voltage = self._convert_data(current_data[1],
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

    def _set_arm_state(self, ystate_arm, arm_state_data,
                       linear_scale, linear_offset,
                       pitch_scale, pitch_offset,
                       yaw_scale, yaw_offset):

        flag_linear, ystate_arm.top_angle = self._convert_data(arm_state_data[0],
                                                               ystate_arm.top_angle,
                                                               linear_scale, linear_offset)
        flag_pitch, ystate_arm.pitch = self._convert_data(arm_state_data[1],
                                                          ystate_arm.pitch,
                                                          pitch_scale, pitch_offset)
        flag_yaw, ystate_arm.yaw = self._convert_data(arm_state_data[2],
                                                      ystate_arm.yaw,
                                                      yaw_scale, yaw_offset)
        ystate_arm.is_ok = flag_linear and flag_pitch and flag_yaw

    def _set_heat_sensor(self, ysensordata_heat, heat_sensor_data, scale, offset):
        ysensordata_heat.is_ok, ysensordata_heat.data = self._convert_data(heat_sensor_data,
                                                                           ysensordata_heat.data,
                                                                           scale, offset)

    def _set_co2_sensor(self, ysensordata_co2, co2_sensor_data, scale, offset):
        ysensordata_co2.is_ok, ysensordata_co2.data = self._convert_data(co2_sensor_data,
                                                                         ysensordata_co2.data,
                                                                         scale, offset)

    def set_data(self, flipper_angles, current_sensor_data, imu_sensor_data, arm_data):
        # print(flipper_angles, current_sensor_data, imu_sensor_data)

        self._set_flipper_angles(self._ystate.base.flipper_left,
                                 flipper_angles[0],
                                 scale=self._flipper_raw2deg, offset=-self._lflipper_center_raw_data)
        self._set_flipper_angles(self._ystate.base.flipper_right,
                                 flipper_angles[1],
                                 scale=-self._flipper_raw2deg, offset=-self._rflipper_center_raw_data)

        lwheel, rwheel, lflip, rflip = current_sensor_data
        self._set_current_sensor_data(self._ysensor_data.wheel_left,
                                      lwheel,
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)
        self._set_current_sensor_data(self._ysensor_data.wheel_right,
                                      rwheel,
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)
        self._set_current_sensor_data(self._ysensor_data.flipper_left,
                                      lflip,
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)
        self._set_current_sensor_data(self._ysensor_data.flipper_right,
                                      rflip,
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)

        front, back = imu_sensor_data
        # もしNoneじゃなかったらdegにする．そうじゃないならNone
        front = [np.rad2deg(data) if data is not None else None for data in front]
        back = [np.rad2deg(data) if data is not None else None for data in back]
        self._set_pose_sensor(self._ystate.base.body_front,
                              front,
                              roll_scale=1.0, roll_offset=0.0,
                              pitch_scale=1.0, pitch_offset=0.0,
                              yaw_scale=1.0, yaw_offset=0.0)

        self._set_pose_sensor(self._ystate.base.body_back,
                              back,
                              roll_scale=1.0, roll_offset=0.0,
                              pitch_scale=1.0, pitch_offset=0.0,
                              yaw_scale=1.0, yaw_offset=0.0)

        arm_state_data, servo_iv, thermo_sensor_data, co2_sensor_data = arm_data
        self._set_arm_state(self._ystate.arm, arm_state_data,
                            linear_scale=self._arm_linear_dxdeg2armdeg, linear_offset=-self._arm_linear_center_dxdeg,
                            pitch_scale=self._arm_pitch_dxdeg2armdeg, pitch_offset=-self._arm_pitch_center_dxdeg,
                            yaw_scale=self._arm_yaw_dxdeg2armdeg, yaw_offset=-self._arm_yaw_center_dxdeg)

        self._set_current_sensor_data(self._ysensor_data.arm_linear,
                                      [0.0, servo_iv[0]],
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)

        self._set_current_sensor_data(self._ysensor_data.arm_pitch,
                                      [servo_iv[1], servo_iv[0]],
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)
        self._set_current_sensor_data(self._ysensor_data.arm_yaw,
                                      [servo_iv[2], servo_iv[0]],
                                      current_scale=1.0, current_offset=0.0,
                                      voltage_scale=1.0, voltage_offset=0.0)
        self._set_heat_sensor(self._ysensor_data.heat, thermo_sensor_data[0] + thermo_sensor_data[1],
                              scale=1.0, offset=0.0)
        self._set_co2_sensor(self._ysensor_data.co2, co2_sensor_data,
                             scale=1.0, offset=0.0)


if __name__ == "__main__":
    rospy.init_node('sensor_data_manager', anonymous=True)
    sensor_data_mgr = SensorDataManager()

    rate_mgr = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown():
        flipper_angle = [0.0, 0.0]  # left, right
        lwheel_current = [0.0, 0.0]  # current, voltage
        rwheel_current = [0.0, 0.0]
        lflipper_current = [0.0, 0.0]
        rflipper_current = [0.0, 0.0]
        front_pose = [0.0, 0.0, 0.0]  # roll, pitch, yaw [rad]
        back_pose = [0.0, 0.0, 0.0]
        arm_state = [0.0, 0.0, 0.0]  # linear, pitch, yaw
        arm_iv = [0.0, 0.0, 0.0]  # voltage, pithc_current, yaw_current
        heat = [[0.0] * 16, [0.0] * 16]
        co2 = 0.0

        # set data
        sensor_data_mgr.set_data(flipper_angle,
                                 (lwheel_current, rwheel_current, lflipper_current, rflipper_current),
                                 (front_pose, back_pose),
                                 (arm_state, arm_iv, heat, co2))

        # this publishes data that is aligned and converted to ros-style data
        sensor_data_mgr.publish_data()
        rate_mgr.sleep()







