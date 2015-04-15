#!/usr/bin/env python
# -*- coding: utf-8 -*-

from common.exceptions import UnknownControllerError

class Axis(object):
    """
    A class representing a controller axis position.

    Parameters
    ----------
    x, y : float
        The positions of the axis..
    inverted : bool, optional
        Whether the direction is y-inverted.

    Attributes
    ----------
    x, y : float
        The positions of the axis..
    inverted : bool
        Whether the direction is y-inverted.

    """

    def __init__(self, x, y, inverted=False):
        self.x = x
        self.y = y
        self.inverted = inverted

    @property
    def direction(self):
        """
        Determine the direction represented by the controller axis position.

        The directions are Up, Down, Left, and Right, and their intermediates.

        Returns
        -------
        str
            "U", "D", "L", and "R" represent Up, Down, Left, and Right
            respectively. Return "U", "D", "L", "R" if the positions are either
            only ``x`` or only ``y``. Otherwise, return "U" or "D", followed by
            "L" or "R", as appropriate. If both the ``x`` and ``y`` positions
            are zero, return "none".

        Examples
        --------
        >>> axis = Axis(0.5, 0.7)
        >>> axis.direction
        'UR'
        >>> axis = Axis(0.9, 0)
        >>> axis.direction
        'R'
        >>> axis = Axis(0, 0)
        >>> axis.direction
        'none'

        """
        if self.y > 0:
            vertical = "D" if self.inverted else "U"
        elif self.y < 0:
            vertical = "U" if self.inverted else "D"
        else:
            vertical = ""

        if self.x > 0:
            horizontal = "R"
        elif self.x < 0:
            horizontal = "L"
        else:
            horizontal = ""

        if not vertical and not horizontal:
            direction = "none"
        else:
            direction = "{}{}".format(vertical, horizontal)

        return direction

    def __repr__(self):
        return str((self.x, self.y))

    def __str__(self):
        return "[{:5.2f}, {:5.2f}]".format(self.x, self.y)


class Buttons(object):
    """
    A class representing the button configuration of a controller.

    Note that this is only tested with the Logitech RumblePad 2. Other input
    devices may have different configurations.

    When registering the mapping for a new controller, please run the
    ``get_name()`` function to obtain the name to use.

    Parameters
    ----------
    buttons : iterable
        A list containing the state of each button. 1 if pressed, 0 otherwise.

    Attributes
    ----------
    buttons : list of int
        A list containing the state of each button.
    pressed_buttons: list of str
        A list containing the names of each button that is pressed.
    known_makes : list of str
        A list containing all the makes whose mappings have been registered.

    Raises
    ------
    UnknownControllerError
        If the mapping of the controller buttons is unknown.

    """
    _button_list = ("□", "✕", "○", "△",  # 0-3
                    "L1", "R1", "L2", "R2",  # 4-7
                    "select", "start",  # 8-9
                    "L3", "R3", "PS")  # 10-12

    _mappings = {"Logitech Logitech RumblePad 2 USB": {},
                 "Elecom Wireless Gamepad": {1: 3, 2: 1, 3: 2}}

    # Populate the mappings.
    for make in _mappings:
        for i in range(13):
            _mappings[make].setdefault(i, i)

    known_makes = list(_mappings.keys())

    def __init__(self, make, buttons):
        if make not in Buttons.known_makes:
            raise UnknownControllerError(make)

        self._make = make
        self.buttons = buttons
        self.pressed = [Buttons._button_list[Buttons._mappings[self._make][i]]
                        for i, button in enumerate(self.buttons) if button]

    def is_pressed(self, button):
        """
        Whether a given button is pressed.

        Parameters
        ----------
        button : str
            The name of the button to be checked.

        Returns
        -------
        bool
            Whether the button is pressed.

        """
        return button in self.pressed

    def all_pressed(self, *buttons):
        """
        Whether all given buttons are pressed.

        Parameters
        ----------
        buttons : one or more str
            The name(s) of the buttons to be checked.

        Returns
        -------
        bool
            True if all the buttons are pressed.

        """
        return all([self.is_pressed(button) for button in buttons])

    def __repr__(self):
        return str(self.buttons)

    def __str__(self):
        return str(self.pressed)