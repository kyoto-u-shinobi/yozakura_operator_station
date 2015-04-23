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
    ARM_MODE_NORMAL = 0
    ARM_MODE_HOME = 1
    ARM_MODE_RESET = 2
    ARM_MODE_SHUTDOWN = 3

    def __init__(self):
        rospy.wait_for_service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME)
        self.input_mode_switch_srv = rospy.ServiceProxy(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME,
                                                        InputModeSwitchService)

        self.input_mode = self.InputMode(1, True, "main", 1)
        self.is_active = False
        self.switched_time = rospy.Time.now()

    def _js_callback(self, joy_data):
        buttons = Buttons(rospy.get_param('~js_maker', DEFAULT_JS_MAKER), joy_data.buttons)

        if joy_data.header.stamp.secs - self.switched_time.secs >= 1.0:
            # switch js_mapping_mode
            if buttons.is_pressed("L3"):
                self.switched_time = joy_data.header.stamp
                self.input_mode_switch_srv(self.input_mode.next_js_mapping_mode,
                                           self.input_mode.direction_flag,
                                           self.input_mode.main_controller_name,
                                           self.input_mode.arm_mode)
            # switch direction_flag
            if buttons.is_pressed("R3"):
                self.switched_time = joy_data.header.stamp
                self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                           self.input_mode.next_direction_flag,
                                           self.input_mode.main_controller_name,
                                           self.input_mode.arm_mode)

            if buttons.is_pressed("start") and buttons.is_pressed("select"):
                self.switched_time = joy_data.header.stamp
                self.input_mode.arm_mode = self.ARM_MODE_SHUTDOWN
                self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                           self.input_mode.direction_flag,
                                           self.input_mode.main_controller_name,
                                           self.input_mode.arm_mode)
            elif buttons.is_pressed("start"):
                self.switched_time = joy_data.header.stamp
                self.input_mode.arm_mode = self.ARM_MODE_HOME
                self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                           self.input_mode.direction_flag,
                                           self.input_mode.main_controller_name,
                                           self.input_mode.arm_mode)
            elif buttons.is_pressed("select"):
                self.switched_time = joy_data.header.stamp
                self.input_mode.arm_mode = self.ARM_MODE_RESET
                self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                           self.input_mode.direction_flag,
                                           self.input_mode.main_controller_name,
                                           self.input_mode.arm_mode)
            else:
                self.switched_time = joy_data.header.stamp
                self.input_mode.arm_mode = self.ARM_MODE_NOMAL
                self.input_mode_switch_srv(self.input_mode.js_mapping_mode,
                                           self.input_mode.direction_flag,
                                           self.input_mode.main_controller_name,
                                           self.input_mode.arm_mode)


    def activate(self):
        self.is_active = True
        rospy.Subscriber(DEFAULT_JOYSTICK_TOPICNAME, Joy, self._js_callback)


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node(DEFAULT_SERVER_NODENAME)
    service_provider = ServiceProvider()
    service_provider.activate()
    rospy.spin()