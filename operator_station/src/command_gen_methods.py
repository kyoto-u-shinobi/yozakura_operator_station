#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Define speed commands calculation functions(this is not class. just a set of functions )
returns:
 - base_vel_input_mode: when this is 1, lwheel and rwheel ∈ [-1, 1].
                        when this is 2, lwheel and rwheel are [m/sec] (this is just for future)
 - lwheel: raw input data for left wheel speed ∈ [-1, 1]
 - rwheel: raw input data for right wheel speed ∈ [-1, 1]
 - lflipper: raw input data for left flipper speed ∈ [-1, 1]
 - rflipper: raw input data for right flipper speed ∈ [-1, 1]

 - arm_mode: 0 is normal, 1 is
 - linear: ∈ [-1, 1]
 - pitch: ∈ [-1, 1]
 - yaw: ∈ [-1, 1]
"""


def calc_speed_command_single_stick_mode(direction_flag, _dpad, _lstick, _rstick, _buttons):
    # convert direction
    if direction_flag:
        dpad, lstick, rstick = _dpad, _lstick, _rstick
        # left upper = lu, right down = rd
        lu_button, ld_button, ru_button, rd_button = 'L1', 'L2', 'R1', 'R2'
    else:
        dpad, lstick, rstick = _dpad.reversed, _lstick.reversed, _rstick.reversed
        lu_button, ld_button, ru_button, rd_button = 'R1', 'R2', 'L1', 'L2'

    base_vel_input_mode = 1

    # Wheels
    if abs(lstick.x) == 0:  # Rotate in place
        lwheel = -lstick.y
        rwheel = lstick.y
    else:
        l_mult = (1 - lstick.y) / (1 + abs(lstick.y))
        r_mult = (1 + lstick.y) / (1 + abs(lstick.y))
        lwheel = lstick.x * l_mult
        rwheel = lstick.x * r_mult

    # Flippers
    if _buttons.is_pressed(lu_button) and (not _buttons.is_pressed(ld_button)):
        lflipper = 1
    elif (not _buttons.is_pressed(lu_button)) and _buttons.is_pressed(ld_button):
        lflipper = -1
    else:
        lflipper = 0

    if _buttons.is_pressed(ru_button) and (not _buttons.is_pressed(rd_button)):
        rflipper = 1
    elif (not _buttons.is_pressed(ru_button)) and _buttons.is_pressed(rd_button):
        rflipper = -1
    else:
        rflipper = 0

    return base_vel_input_mode, lwheel, rwheel, lflipper, rflipper


def calc_speed_command_dual_stick_mode(direction_flag, _dpad, _lstick, _rstick, _buttons):
    # convert direction
    if direction_flag:
        dpad, lstick, rstick = _dpad, _lstick, _rstick
        # left upper = lu, right down = rd
        lu_button, ld_button, ru_button, rd_button = 'L1', 'L2', 'R1', 'R2'
    else:
        dpad, lstick, rstick = _dpad.reversed, _rstick.reversed, _lstick.reversed
        lu_button, ld_button, ru_button, rd_button = 'R1', 'R2', 'L1', 'L2'

    base_vel_input_mode = 1

    # Wheels
    lwheel = lstick.x
    rwheel = rstick.x

    # Flippers
    if _buttons.is_pressed(lu_button) and (not _buttons.is_pressed(ld_button)):
        lflipper = 1
    elif (not _buttons.is_pressed(lu_button)) and _buttons.is_pressed(ld_button):
        lflipper = -1
    else:
        lflipper = 0

    if _buttons.is_pressed(ru_button) and (not _buttons.is_pressed(rd_button)):
        rflipper = 1
    elif (not _buttons.is_pressed(ru_button)) and _buttons.is_pressed(rd_button):
        rflipper = -1
    else:
        rflipper = 0

    return base_vel_input_mode, lwheel, rwheel, lflipper, rflipper


def calc_arm_command_default_mode(direction_flag, _dpad, _lstick, _rstick, _buttons):
    if _buttons.is_pressed("○"):
        linear, pitch, yaw = _dpad.x, 0.0, 0.0
    else:
        linear, pitch, yaw = 0.0, _dpad.x, _dpad.y

    if _buttons.all_pressed('start', 'select'):
        arm_mode = 3
        linear, pitch, yaw = 0.0, 0.0, 0.0
    elif _buttons.is_pressed('start'):
        arm_mode = 1
        linear, pitch, yaw = 0.0, 0.0, 0.0
    elif _buttons.is_pressed('select'):
        arm_mode = 2
        linear, pitch, yaw = 0.0, 0.0, 0.0
    else:
        arm_mode = 0

    return arm_mode, linear, pitch, yaw
