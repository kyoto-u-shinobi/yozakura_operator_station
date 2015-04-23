#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from js_state import Buttons
from operator_station.srv import InputModeSwitchService

# remap-able names
DEFAULT_SERVER_NODENAME = "service_provider"
DEFAULT_JOYSTICK_TOPICNAME = 'joy'
DEFAULT_INPUT_MODE_SWITCH_SERVICENAME = "input_mode_switcher"
DEFAULT_THETA_SHUTTER_SERVICENAME = "theta_capture"

# param-able
DEFAULT_JS_MAKER = "Elecom Wireless Gamepad"


class ServiceProvider(object):
    class InputMode():
        def __init__(self, js_mapping_mode=1, direction_flag=True, main_controller_name="main",
                     arm_mode=ARM_MODE_NORMAL):
            self.js_mapping_mode = js_mapping_mode
            self.js_mapping_mode_max = 2  # 1: single-stick mode 2: dual-stick mode
            self.direction_flag = direction_flag
            self.main_controller_name = main_controller_name
            self.arm_mode = arm_mode

        @property
        def next_js_mapping_mode(self):
            self.js_mapping_mode = (self.js_mapping_mode % self.js_mapping_mode_max) + 1
            return self.js_mapping_mode

        @property
        def next_direction_flag(self):
            self.direction_flag = not self.direction_flag
            return self.direction_flag


    def __init__(self):
        rospy.wait_for_service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME)
        self.input_mode_switch_srv = rospy.ServiceProxy(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME,
                                                        InputModeSwitchService)

        self.input_mode = self.InputMode(1, True, "main", 1)
        self.is_active = False
        self._base_mode_switch_time = rospy.Time.now()

        self._arm_modes_dict = {
            'normal': [0, rospy.Time.now(), False],
            'home': [1, rospy.Time.now(), False],
            'reset': [2, rospy.Time.now(), False],
            'shutdown': [3, rospy.Time.now(), False]
        }

    def _switch_base_control_modes(self, time_stamp, buttons):
        # switch js_mapping_mode
        if buttons.is_pressed("L3"):
            self._base_mode_switch_time = time_stamp
            self.input_mode_switch_srv(self.input_mode.next_js_mapping_mode,
                                       self.input_mode.direction_flag,
                                       self.input_mode.main_controller_name,
                                       self.input_mode.arm_mode)
        # switch direction_flag
        if buttons.is_pressed("R3"):
            self._base_mode_switch_time = time_stamp
            self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                       self.input_mode.next_direction_flag,
                                       self.input_mode.main_controller_name,
                                       self.input_mode.arm_mode)

    def _switch_arm_control_modes(self, time_stamp, buttons):
        def filter4arm_mode(_key, _wait_sec):
            for key in self._arm_modes_dict.keys():
                self._arm_modes_dict[key][2] = False if key is not _key else self._arm_modes_dict[key][2]

            if self._arm_modes_dict[_key][2] is False:
                self._arm_modes_dict[_key][2] = True
                self._arm_modes_dict[_key][1] = time_stamp
            else:
                if time_stamp.secs - self._arm_modes_dict[_key][1].secs >= _wait_sec:
                    self.input_mode.arm_mode = self._arm_modes_dict[_key][0]
                    self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                               self.input_mode.direction_flag,
                                               self.input_mode.main_controller_name,
                                               self.input_mode.arm_mode)

        if buttons.is_pressed("start") and buttons.is_pressed("select"):
            filter4arm_mode('shutdown', 3.0)
        elif buttons.is_pressed("start"):
            filter4arm_mode('home', 1.0)
        elif buttons.is_pressed("select"):
            filter4arm_mode('reset', 3.0)
        else:
            for key in self._arm_modes_dict.keys():
                self._arm_modes_dict[key][2] = False


    def _js_callback(self, joy_data):
        buttons = Buttons(rospy.get_param('~js_maker', DEFAULT_JS_MAKER), joy_data.buttons)

        if joy_data.header.stamp.secs - self._base_mode_switch_time.secs >= 1.0:
            self._switch_base_control_modes(joy_data.header.stamp, buttons)

        self._switch_arm_control_modes(self, joy_data.header.stamp, buttons)

    def activate(self):
        self.is_active = True
        rospy.Subscriber(DEFAULT_JOYSTICK_TOPICNAME, Joy, self._js_callback)


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node(DEFAULT_SERVER_NODENAME)
    service_provider = ServiceProvider()
    service_provider.activate()
    rospy.spin()