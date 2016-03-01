#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging

import rospy
from js_controller import JoyStickController
from yozakura_msgs.msg import YozakuraCommand
from yozakura_msgs.srv import InputModeSwitchService, InputModeSwitchServiceResponse

import command_gen_methods as methods

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
        self.arm_mode = 0

        self._calc_speed_command_funcs = [
            methods.calc_speed_command_single_stick_mode,
            methods.calc_speed_command_dual_stick_mode
        ]
        self._calc_arm_command_funcs = [
            methods.calc_arm_command_default_mode
        ]

    def add_controller(self, topic_name, controller_name=None):
        controller = JoyStickController(topic_name, controller_name)
        self.js_controllers[controller.name] = controller

    def remove_controller(self, name):
        del self.js_controllers[name]

    def switch_main_controller(self, name):
        self.main_controller_name = name

    def _input_mode_switching_handler(self, req):
        print(req)
        self.js_mapping_mode = req.js_mapping_mode
        self.direction_flag = req.direction_flag
        self.main_controller_name = req.main_controller_name
        return InputModeSwitchServiceResponse()

    def activate(self):
        # build server
        rospy.Service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME, InputModeSwitchService, self._input_mode_switching_handler)

        # run publishers for getting joystick data
        for controller in self.js_controllers.values():
            if controller.is_active is not True:
                controller.activate()

    def publish_command(self):
        base_vel_input_mode, vel1, vel2 = self.get_speed_commands()

        arm_mode, wrist_yaw, elbow_pitch, base_pitch, base_yaw = self.get_arm_commands()

        # self._ycommand.header.stamp = rospy.get_time()

        self._ycommand.arm_vel.is_ok = True
        self._ycommand.arm_vel.mode = arm_mode
        self._ycommand.arm_vel.wrist_yaw = wrist_yaw
        self._ycommand.arm_vel.elbow_pitch = elbow_pitch
        self._ycommand.arm_vel.base_pitch = base_pitch
        self._ycommand.arm_vel.base_yaw = base_yaw

        self._ycommand.base_vel_input_mode = base_vel_input_mode
        if base_vel_input_mode == 1:
            self._ycommand.wheel_left_vel = vel1
            self._ycommand.wheel_right_vel = vel2
            self._ycommand.base_vel.linear.x = 0.0
            self._ycommand.base_vel.angular.z = 0.0
        elif base_vel_input_mode == 2:
            self._ycommand.wheel_left_vel = 0.0
            self._ycommand.wheel_right_vel = 0.0
            self._ycommand.base_vel.linear.x = vel1
            self._ycommand.base_vel.angular.z = vel2
        else:
            self._ycommand.wheel_left_vel = 0.0
            self._ycommand.wheel_right_vel = 0.0
            self._ycommand.base_vel.linear.x = 0.0
            self._ycommand.base_vel.angular.z = 0.0

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
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return self._calc_speed_command_funcs[self.js_mapping_mode - 1](self.direction_flag,
                                                                        dpad, lstick, rstick, buttons)

    def get_arm_commands(self):
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return self._calc_arm_command_funcs[0](self.direction_flag,
                                               dpad, lstick, rstick, buttons)

# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('command_generator', anonymous=True)
    cmd_gen = CommandGenerator(logging, 'joy')
    cmd_gen.activate()

    rate_mgr = rospy.Rate(30)  # hz
    while not rospy.is_shutdown():
        # print(cmd_gen.get_jsstate())
        # print(cmd_gen.get_input())
        # print(cmd_gen.get_speed_commands())
        cmd_gen.publish_command()
        rate_mgr.sleep()


