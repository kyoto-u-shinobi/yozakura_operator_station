#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from js_state import Buttons
from operator_station.srv import InputModeSwitchService

# remap-able names
DEFAULT_JOYSTICK_TOPICNAME = 'joy'
DEFAULT_INPUT_MODE_SWITCH_SERVICENAME = "input_mode_switcher"
DEFAULT_THETA_SHUTTER_SERVICENAME = "theta_capture"
# param-able
DEFAULT_JS_MAKER = "Elecom Wireless Gamepad"


class ServiceProvider(object):
    class InputMode():
        def __init__(self, js_mapping_mode=1, direction_flag=True, main_controller_name="main"):
            self.js_mapping_mode = js_mapping_mode
            self.js_mapping_mode_max = 2  # 1: single-stick mode 2: dual-stick mode
            self.direction_flag = direction_flag
            self.main_controller_name = main_controller_name

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

        self.input_mode = self.InputMode(1, True, "main")
        self.is_active = False
        self.current_time = rospy.Time.now()

    def _js_callback(self, joy_data):
        buttons = Buttons(rospy.get_param('~js_maker', DEFAULT_JS_MAKER), joy_data.buttons)

        if self.current_time - joy_data.header.stamp >= 1.0:
            self.current_time = joy_data.header.stamp
            # switch js_mapping_mode
            if buttons.is_pressed("L3"):
                self.input_mode_switch_srv(self.input_mode.next_js_mapping_mode,
                                           self.input_mode.direction_flag,
                                           self.input_mode.main_controller_name)

            # switch direction_flag
            if buttons.is_pressed("R3"):
                self.current_time = joy_data.header.stamp
                self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                           self.input_mode.next_direction_flag,
                                           self.input_mode.main_controller_name)

    def activate(self):
        self.is_active = True
        rospy.Subscriber(DEFAULT_JOYSTICK_TOPICNAME, Joy, self._js_callback)



