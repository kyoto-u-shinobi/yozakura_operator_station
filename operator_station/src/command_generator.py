#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging

import rospy
from js_controller import JoyStickController
from operator_station.srv import InputModeSwitchService

# remap-able names
DEFAULT_INPUT_MODE_SWITCH_SERVICENAME = "input_mode_switcher"


class CommandGenerator(object):
    def __init__(self, logger, main_jstopic_name, main_controller_name=None):
        self._logger = logger
        self.js_controllers = {}
        self.add_controller(main_jstopic_name, main_controller_name)

        # see: srv/InputModeSwitchService.srv
        self.js_mapping_mode = 1
        self.direction_flag = True
        self._culc_speed_command_funcs = [
            self._calc_speed_command_single_stick_mode,
            self._calc_speed_command_dual_stick_mode
        ]

        if main_controller_name is not None:
            self.main_controller_name = main_controller_name
        else:
            self.main_controller_name = "main"

    def add_controller(self, topic_name, controller_name=None):
        controller = JoyStickController(topic_name, controller_name)
        self.js_controllers[controller.name] = controller

    def remove_controller(self, name):
        del self.js_controllers[name]

    def switch_main_controller(self, name):
        self.main_controller_name = name

    def _input_mode_switching_handler(self, req):
        self.js_mapping_mode = req.js_mapping_mode
        self.direction_mode = req.direction_mode
        self.main_controller_name = req.main_controller_name

    def activate(self):
        # run publishers for getting joystick data
        for controller in self.js_controllers:
            if not controller.is_active: controller.activate()
        # build server
        rospy.Service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME, InputModeSwitchService, self._input_mode_switching_handler)

    def get_jsstate(self):
        return self.js_controllers[self.main_controller_name].state

    def get_input(self):
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return ((dpad.x, dpad.y),
                (lstick.x, lstick.y),
                (rstick.x, rstick.y),
                buttons.buttons)

    def get_speed_commands(self):
        return self._culc_speed_command_funcs[self.js_mapping_mode - 1](self.direction_mode)


    def _calc_speed_command_single_stick_mode(self, direction_flag):
        _dpad, _lstick, _rstick, buttons = self.get_jsstate()

        # convert direction
        if direction_flag:
            dpad, lstick, rstick = _dpad, _lstick, _rstick
        else:
            dpad, lstick, rstick = _dpad.reversed, _lstick.reversed, _rstick.reversed

        self._logger.debug("lx: {lx:9.7}  ly: {ly:9.7}".format(lx=lstick.x, ly=lstick.y))

        # Wheels
        if abs(lstick.y) == 0:  # Rotate in place
            lwheel = lstick.x
            rwheel = -lstick.x
        else:
            l_mult = (1 + lstick.x) / (1 + abs(lstick.x))
            r_mult = (1 - lstick.x) / (1 + abs(lstick.x))
            lwheel = -lstick.y * l_mult
            rwheel = -lstick.y * r_mult

        # Flippers
        if buttons.all_pressed("L1", "L2"):
            lflipper = 0
        elif buttons.is_pressed("L1"):
            lflipper = 1
        elif buttons.is_pressed("L2"):
            lflipper = -1
        else:
            lflipper = 0

        if buttons.all_pressed("R1", "R2"):
            rflipper = 0
        elif buttons.is_pressed("R1"):
            rflipper = 1
        elif buttons.is_pressed("R2"):
            rflipper = -1
        else:
            rflipper = 0

        return lwheel, rwheel, lflipper, rflipper


    def _calc_speed_command_dual_stick_mode(self, direction_flag):
        _dpad, _lstick, _rstick, buttons = self.get_jsstate()

        # convert direction
        if direction_flag:
            dpad, lstick, rstick = _dpad, _lstick, _rstick
        else:
            dpad, lstick, rstick = _dpad.reversed, _lstick.reversed, _rstick.reversed

        self._logger.debug("lx: {lx:9.7}  ly: {ly:9.7}".format(lx=lstick.x, ly=lstick.y))

        # Wheels
        lwheel = rstick.y
        rwheel = lstick.y

        # Flippers
        if buttons.all_pressed("L1", "L2"):
            lflipper = 0
        elif buttons.is_pressed("L1"):
            lflipper = 1
        elif buttons.is_pressed("L2"):
            lflipper = -1
        else:
            lflipper = 0

        if buttons.all_pressed("R1", "R2"):
            rflipper = 0
        elif buttons.is_pressed("R1"):
            rflipper = 1
        elif buttons.is_pressed("R2"):
            rflipper = -1
        else:
            rflipper = 0

        return lwheel, rwheel, lflipper, rflipper


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('command_generator', anonymous=True)
    cmd_gen = CommandGenerator(logging, 'joy')
    cmd_gen.activate()

    rate_mgr = rospy.Rate(10)  # hz
    while not rospy.is_shutdown():
        print(cmd_gen.get_jsstate())
        print(cmd_gen.get_input())
        print(cmd_gen.get_speed_commands())
        rate_mgr.sleep()


