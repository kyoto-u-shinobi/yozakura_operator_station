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
    """
    Generate command for yozakura
    Subscribe: None
    Publish: YozakuraCommand
    Service Server: InputModeSwitchService
    """
    def __init__(self, main_jstopic_name):
        self.js_controller = JoyStickController(main_jstopic_name)

        self._ycommand = YozakuraCommand()
        self._pub_command = rospy.Publisher(DEFAULT_PUB_COMMAND_TOPICNAME, YozakuraCommand, queue_size=10)

        # see: srv/InputModeSwitchService.srv
        self.js_mapping_mode = 1
        self.direction_flag = True
        # TODO: add self.arm_mode to InputModeSwitchService (if necessary)
        self.arm_mode = 0

        self._calc_speed_command_funcs = [
            methods.calc_speed_command_single_stick_mode,
            methods.calc_speed_command_dual_stick_mode
        ]
        self._calc_arm_command_funcs = [
            methods.calc_arm_command_default_mode
        ]

    def _input_mode_switching_handler(self, request):
        """
        Called when InputModeSwitchService is requested. Study ROS!
        :param request: See InputModeSwitchService.srv
        :return: InputModeSwitchServiceResponse
        """
        print(request)
        self.js_mapping_mode = request.js_mapping_mode
        self.direction_flag = request.direction_flag
        self.main_controller_name = request.main_controller_name
        return InputModeSwitchServiceResponse()

    def activate(self):
        """
        Build server and run publisher for getting joystick data
        """
        rospy.Service(DEFAULT_INPUT_MODE_SWITCH_SERVICENAME, InputModeSwitchService, self._input_mode_switching_handler)

        if self.js_controller.is_active is not True:
            self.js_controller.activate()

    def publish_command(self):
        """
        Publish YozakuraCommand
        """
        self._update_yozakura_command(self._ycommand)

        self._pub_command.publish(self._ycommand)

    def get_jsstate(self):
        """
        Getter
        :return: JoyStickController.data
        """
        return self.js_controller.state.data

    def get_input(self):
        """
        Getter
        :return: joystick state
        """
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return ((dpad.x, dpad.y),
                (lstick.x, lstick.y),
                (rstick.x, rstick.y),
                buttons.buttons)

    def get_speed_commands(self):
        """
        Choose the command generation method from self._calc_speed_command_funcs
        if base_vel_input_mode is 1, vel1 and vel2 are left and right wheel velocity ∈ [-1, 1]
        if base_vel_input_mode is 2, vel1 and vel2 are v and w [m/s] (just for the future)
        :return:base_vel_input_mode, vel1, vel2, lflipper, rflipper
        """
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return self._calc_speed_command_funcs[self.js_mapping_mode - 1](self.direction_flag,
                                                                        dpad, lstick, rstick, buttons)

    def get_arm_commands(self):
        """
        Choose the command generation method from self._calc_arm_command_funcs
        :return:arm_mode, linear, pitch, yaw
        """
        dpad, lstick, rstick, buttons = self.get_jsstate()
        return self._calc_arm_command_funcs[self.arm_mode](self.direction_flag,
                                               dpad, lstick, rstick, buttons)

    def _update_yozakura_command(self, yozakura_command):
        base_vel_input_mode, vel1, vel2, lflipper, rflipper = self.get_speed_commands()
        arm_mode, linear, pitch, yaw = self.get_arm_commands()

        yozakura_command.header.stamp = rospy.Time.now()

        yozakura_command.arm_vel.is_ok = True
        yozakura_command.arm_vel.mode = arm_mode
        yozakura_command.arm_vel.top_angle = linear
        yozakura_command.arm_vel.pitch = pitch
        yozakura_command.arm_vel.yaw = yaw

        yozakura_command.base_vel_input_mode = base_vel_input_mode
        if base_vel_input_mode == 1:
            yozakura_command.wheel_left_vel = vel1
            yozakura_command.wheel_right_vel = vel2
            yozakura_command.base_vel.linear.x = 0.0
            yozakura_command.base_vel.angular.z = 0.0
        elif base_vel_input_mode == 2:
            yozakura_command.wheel_left_vel = 0.0
            yozakura_command.wheel_right_vel = 0.0
            yozakura_command.base_vel.linear.x = vel1
            yozakura_command.base_vel.angular.z = vel2
        else:
            yozakura_command.wheel_left_vel = 0.0
            yozakura_command.wheel_right_vel = 0.0
            yozakura_command.base_vel.linear.x = 0.0
            yozakura_command.base_vel.angular.z = 0.0

        yozakura_command.flipper_left_vel.is_ok = True
        yozakura_command.flipper_left_vel.angle = lflipper
        yozakura_command.flipper_right_vel.is_ok = True
        yozakura_command.flipper_right_vel.angle = rflipper

        return yozakura_command

# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('command_generator', anonymous=True)
    cmd_gen = CommandGenerator('joy')
    cmd_gen.activate()

    rate_mgr = rospy.Rate(30)  # hz
    while not rospy.is_shutdown():
        cmd_gen.publish_command()
        rate_mgr.sleep()


