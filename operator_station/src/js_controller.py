#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from js_state import Axis, Buttons, State
from sensor_msgs.msg import Joy

# remap-able
DEFAULT_NODE_NAME = "js_controller"
DEFAULT_TOPIC_NAME = "joy"
# param-able
DEFAULT_JS_MAKER = "Elecom Wireless Gamepad"


class JoyStickController(object):
    """
    ジョイスティックのデータの構造体的な感じで使う
    Subscribe: joy
    Publish: None
    Parameters
    ----------
    dpad: Axis class. 十字キー (x: vertical, y: horizontal)
    lstick: Axis class. 左スティック (x: vertical, y: horizontal)
    rstick: Axis class. 右スティック (x: vertical, y: horizontal)
    buttons: Buttons class.
    """
    def __init__(self, topic_name):
        self.dpad = Axis(0.0, 0.0)
        self.lstick = Axis(0.0, 0.0)
        self.rstick = Axis(0.0, 0.0)
        self.buttons = Buttons(rospy.get_param('~js_maker', DEFAULT_JS_MAKER), [0] * 13)

        self.is_active = False
        self._topic_name = topic_name

    def joy_callback(self, joy_data):
        """
        Callback for the subscriber
        :param joy_data: data from joy_node
        :return:None
        """
        self.dpad.set_data(joy_data.axes[5], joy_data.axes[4])
        self.lstick.set_data(joy_data.axes[1], joy_data.axes[0])
        self.rstick.set_data(joy_data.axes[3], joy_data.axes[2])
        self.buttons.set_data(joy_data.buttons)

    def activate(self):
        self.is_active = True
        rospy.Subscriber(self._topic_name, Joy, self.joy_callback)

    @property
    def state(self):
        return State(self.dpad, self.lstick, self.rstick, self.buttons)


# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    js_controller = JoyStickController(DEFAULT_TOPIC_NAME)
    js_controller.activate()

    r = rospy.Rate(10)  # hz
    while not rospy.is_shutdown():
        print(js_controller.state)
        r.sleep()


