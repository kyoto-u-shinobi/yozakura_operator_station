#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging

import rospy
from js_controller import JoyStickController
from yozakura_msgs.msg import YozakuraCommand
from operator_station.srv import InputModeSwitchService

# remap-able names
DEFAULT_INPUT_MODE_SWITCH_SERVICENAME = "input_mode_switcher"
DEFAULT_PUB_COMMAND_TOPICNAME = "yozakura_command"


class CommandGenerator(object):
    def __init__(self, logger, main_jstopic_name, main_controller_name="main"):
        self._logger = logger
        self.main_controller_name = main_controller_name
        self.js_controllers = {}
        self.add_controller(main_jstopic_name, self.main_controller_name)

        self._ycommand = YozakuraCommand()
        self._pub_command = rospy.Publisher(DEFAULT_PUB_COMMAND_TOPICNAME, YozakuraCommand, queue_size=10)

        # see: srv/InputModeSwitchService.srv
        self.js_mapping_mode = 1
        self.direction_flag = True
        self._culc_speed_command_funcs = [
            self._calc_speed_command_single_stick_mode,
            self._calc_speed_command_dual_stick_mode
        ]


    def add_controller(self, topic_name, controller_name=None):
        controller = JoyStickController(topic_name, controller_name)
        self.js_controllers[controller.name] = controller

    def remove_controller(self, name):
        del self.js_controllers[name]

    def switch_main_controller(self, name):
        self.main_controller_name = name

    def _input_mode_switching_handler(self, req):
        self.js_mapping_mode = req.js_mapping_mode
        self.direction_flag = req.direction_flag
        self.main_controller_name = req.main_controller_name

    def activate(self):
        # build server
        rospy.Service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME, InputModeSwitchService, self._input_mode_switching_handler)
        # run publishers for getting joystick data
        for controller in self.js_controllers.values():
            if controller.is_active is not True:
                controller.activate()

    def publish_command(self):
        v, w, lflipper, rflipper = self.get_speed_commands()

        # self._ycommand.header.stamp = rospy.get_time()

        self._ycommand.arm_vel.is_ok = True
        self._ycommand.arm_vel.top_angle = 0.0
        self._ycommand.arm_vel.pitch = 0.0
        self._ycommand.arm_vel.yaw = 0.0

        self._ycommand.base_vel.linear.x = v
        self._ycommand.base_vel.angular.z = w

        self._ycommand.flipper_left_vel.is_ok = True
        self._ycommand.flipper_left_vel.angle = lflipper
        self._ycommand.flipper_right_vel.is_ok = True
        self._ycommand.flipper_right_vel.angle = rflipper

        self._pub_command.publish(self._ycommand)

    def get_jsstate(self, controller_name=None):
        if controller_name is not None:
            self.main_controller_name = controller_name
        return self.js_controllers[self.main_controller_name].state.data

    def get_input(self):
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return ((dpad.x, dpad.y),
                (lstick.x, lstick.y),
                (rstick.x, rstick.y),
                buttons.buttons)

    def get_speed_commands(self):
        return self._culc_speed_command_funcs[self.js_mapping_mode - 1](self.direction_flag)


    def _calc_speed_command_single_stick_mode(self, direction_flag):
        _dpad, _lstick, _rstick, buttons = self.get_jsstate()

        # convert direction
        if direction_flag:
            dpad, lstick, rstick = _dpad, _lstick, _rstick
        else:
            dpad, lstick, rstick = _dpad.reversed, _lstick.reversed, _rstick.reversed

        self._logger.debug("lx: {lx:9.7}  ly: {ly:9.7}".format(lx=lstick.x, ly=lstick.y))

        # Wheels
        v = lstick.y
        w = lstick.x
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

        return v, w, lflipper, rflipper


    def _calc_speed_command_dual_stick_mode(self, direction_flag):
        _dpad, _lstick, _rstick, buttons = self.get_jsstate()

        # convert direction
        if direction_flag:
            dpad, lstick, rstick = _dpad, _lstick, _rstick
        else:
            dpad, lstick, rstick = _dpad.reversed, _lstick.reversed, _rstick.reversed

        self._logger.debug("lx: {lx:9.7}  ly: {ly:9.7}".format(lx=lstick.x, ly=lstick.y))

        # Wheels
        v = (rstick.y + lstick.y) / 2.0
        w = (rstick.y - lstick.y) / 2.0

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

        return v, w, lflipper, rflipper


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
        cmd_gen.publish_command()
        rate_mgr.sleep()


