#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from js_state import Buttons
from yozakura_msgs.msg import InputMode
from yozakura_msgs.srv import InputModeSwitchService

# remap-able names
DEFAULT_SERVER_NODENAME = "service_provider"
DEFAULT_JOYSTICK_TOPICNAME = 'joy'
DEFAULT_INPUT_MODE_TOPICNAME = "input_mode"
DEFAULT_INPUT_MODE_SWITCH_SERVICENAME = "input_mode_switcher"
DEFAULT_THETA_SHUTTER_SERVICENAME = "theta_capture"

# param-able
DEFAULT_JS_MAKER = "Elecom Wireless Gamepad"


class ServiceProvider(object):
    """
    Yozakuraで使うROSserviceを扱う
    """
    class InputMode():
        """
        InputModeSwitchServiceのリクエストを扱う
        """
        def __init__(self, js_mapping_mode=1, direction_flag=True, main_controller_name="main"):
            self.input_mode = InputMode()
            self.input_mode.js_mapping_mode = js_mapping_mode
            self.input_mode.direction_flag = direction_flag
            self.input_mode.main_controller_name = main_controller_name

            self.js_mapping_mode_max = 2

        @property
        def next_js_mapping_mode(self):
            self.input_mode.js_mapping_mode = (self.input_mode.js_mapping_mode % self.js_mapping_mode_max) + 1
            return self.input_mode.js_mapping_mode

        @property
        def next_direction_flag(self):
            self.input_mode.direction_flag = not self.input_mode.direction_flag
            return self.input_mode.direction_flag

        @property
        def js_mapping_mode(self):
            return self.input_mode.js_mapping_mode

        @property
        def direction_flag(self):
            return self.input_mode.direction_flag

        @property
        def main_controller_name(self):
            return self.input_mode.main_controller_name


    def __init__(self):
        rospy.wait_for_service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME)
        self.pub_current_mode = rospy.Publisher(DEFAULT_INPUT_MODE_TOPICNAME, InputMode, queue_size=10)
        self.input_mode_switch_srv = rospy.ServiceProxy(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME,
                                                        InputModeSwitchService)

        self.input_mode = self.InputMode(1, True, "main")
        self.is_active = False
        self._base_mode_switch_time = rospy.Time.now()
        self._js_maker_name = rospy.get_param('~js_maker', DEFAULT_JS_MAKER)

        self.pub_current_mode.publish(self.input_mode.input_mode)

    def _switch_base_control_modes(self, time_stamp, buttons):
        """
        Switch input_mode
        :param time_stamp: ROS original type
        :param buttons: Button class(defined in js_state.py)
        :return:
        """

        # switch js_mapping_mode
        if buttons.is_pressed("L3"):
            self._base_mode_switch_time = time_stamp
            self.input_mode_switch_srv(self.input_mode.next_js_mapping_mode,
                                       self.input_mode.direction_flag,
                                       self.input_mode.main_controller_name)
        # switch direction_flag
        if buttons.is_pressed("R3"):
            self._base_mode_switch_time = time_stamp
            self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                       self.input_mode.next_direction_flag,
                                       self.input_mode.main_controller_name)
        self.pub_current_mode.publish(self.input_mode.input_mode)

    def _js_callback(self, joy_data):
        buttons = Buttons(self._js_maker_name, joy_data.buttons)

        # 連打を防ぐ
        if joy_data.header.stamp.secs - self._base_mode_switch_time.secs >= 1.0:
            self._switch_base_control_modes(joy_data.header.stamp, buttons)

    def activate(self):
        self.is_active = True
        rospy.Subscriber(DEFAULT_JOYSTICK_TOPICNAME, Joy, self._js_callback)


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node(DEFAULT_SERVER_NODENAME)
    service_provider = ServiceProvider()
    service_provider.activate()
    rospy.spin()