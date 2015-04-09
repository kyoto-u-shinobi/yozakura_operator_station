#!/usr/bin/env python
# -*- coding: utf-8 -*-

# (C) 2015  Kyoto University Mechatronics Laboratory
# Released under the GNU General Public License, version 3
"""
Implement a controller in order to control the robot.

Provides classes for directional position, button states, and full controller
state, as well as a ``Controller`` class.

"""

import logging
from controller_commands import Axis, Buttons
import rospy
from sensor_msgs.msg import Joy


class State(object):
    """
    The state of the object.

    Parameters
    ----------
    dpad : Axis
        The position of the dpad.
    lstick : Axis
        The position of the left analog stick.
    rstick : Axis
        The position of the right analog stick.
    buttons : Buttons
        The state of the buttons.

    Attributes
    ----------
    dpad : Axis
        The position of the dpad.
    lstick : Axis
        The position of the left analog stick.
    rstick : Axis
        The position of the right analog stick.
    buttons : Buttons
        The state of the buttons.

    """

    def __init__(self, dpad, lstick, rstick, buttons):
        self.dpad = dpad
        self.lstick = lstick
        self.rstick = rstick
        self.buttons = buttons

    @property
    def data(self):
        """
        Return the raw data.

        Returns
        -------
        dpad, lstick, rstick : 2-tuple of float
            The positions of the dpad and the left and right analog sticks.
        buttons : list of int
            The list of buttons. 1 if pressed, 0 otherwise.

        """
        return self.dpad, self.lstick, self.rstick, self.buttons

    def __repr__(self):
        return str(self.data)

    def __str__(self):
        """
        A human-readable representation of the state.

        To print on a single line, ensure that the terminal is at least 144
        characters wide, and end your `print` function with a carriage return
        character to go back to the start of the line.

        Returns
        -------
        str
            A string with a maximum length of 144 characters, showing the
            positions of the dpad, and left and right analog sticks; as well
            as a list showing all the buttons that are currently pressed.

        Examples
        --------
        >>> stick = Controller(0, "body")
        >>> try:
        ...     While True:  # Below, end="backslash r"
        ...         print(stick_body.get_state(), end="\r")
        ... except (KeyboardInterrupt, SystemExit):  # Exit safely.
        ...     Controller.shutdown_all()
        dpad: UR   lstick: [-1.00,  0.00]  rstick: [ 0.12, -0.45]  buttons: []

        """
        out_1 = "dpad: {:4}".format(self.dpad.direction)
        out_2 = "lstick: {}".format(self.lstick)
        out_3 = "rstick: {}".format(self.rstick)
        out_4 = "buttons: {:75}".format(str(self.buttons))
        return "{}  {}  {}  {}".format(out_1, out_2, out_3, out_4)


class Controller(object):
    """
    A controller to control the robot.

    The controller wraps a pygame Joystick object.

    Parameters
    ----------
    stick_id : int
        The ID of the controller.
    name : str, optional
        The name of the controller.

    Attributes
    ----------
    stick_id : int
        The ID of the controller.
    """
    def __init__(self, stick_id, name=None):
        self._logger = logging.getLogger("controller-{}".format(stick_id))
        self._logger.debug("Initializing controller")
        self.name = name if not None else 'joystick'

        self.initialize_js_data()
        rospy.init_node('controller', anonymous=True)
        rospy.Subscriber('joy', Joy, self.js_callback)

        self._logger.info("Controller initialized")

    def initialize_js_data(self):
        self.hat = {'x': 0, 'y': 0}
        self.lstick = {'x': 0, 'y': 0}
        self.rstick = {'x': 0, 'y': 0}
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


    def js_callback(self, joy_data):
        self.hat['x'] = joy_data.axes[5]
        self.hat['y'] = joy_data.axes[4]
        self.lstick['x'] = joy_data.axes[2]
        self.lstick['y'] = joy_data.axes[1]
        self.rstick['x'] = joy_data.axes[4]
        self.rstick['y'] = joy_data.axes[3]
        self.buttons = joy_data.buttons

    def get_state(self):
        """
        Read the state of all the inputs of the controller.

        Note that this is only tested with the Logitech RumblePad 2. Other
        input devices may have different configurations.

        Returns
        -------
        State
            The controller state.
        """

        self._logger.debug("Getting state")
        dpad = Axis(self.hat['x'], self.hat['y'])
        lstick = Axis(self.lstick['x'], self.lstick['y'], inverted=True)
        rstick = Axis(self.rstick['x'], self.rstick['y'], inverted=True)
        buttons = Buttons(self.make, self.buttons)

        return State(dpad, lstick, rstick, buttons)

    def __repr__(self):
        return "{} (ID# {})".format(self.name, self.stick_id())

    def __str__(self):
        return self.name


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    stick_body = Controller(0, "Body controller")

    while True:
        try:
            print(stick_body.get_state() + "\r")
        except (KeyboardInterrupt, SystemExit):  # Exit safely.
            logging.info("")
            logging.info("Exiting")
            break
