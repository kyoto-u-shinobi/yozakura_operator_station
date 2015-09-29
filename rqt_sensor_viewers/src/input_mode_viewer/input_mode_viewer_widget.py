#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os, time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QWidget
import rospkg
import rospy
from rospy.exceptions import ROSException
from yozakura_msgs.msg import InputMode

DEFAULT_TOPIC_NAME = 'input_mode'


class InputModeViewerWidget(QWidget):
    class ExtendedInputMode():
        """
        ROSserviceのInputModeの拡張クラス
        名前がよくない
        """
        def __init__(self):
            self.input_mode = InputMode()
            self.input_mode.js_mapping_mode = 1
            self.input_mode.direction_flag = True
            self._js_mapping_mode_labels = ['Single-stick', 'Dual-stick']
            self._direction_labels = ['Forward', 'Backward']

        def set_input_mode(self, input_mode):
            self.input_mode = input_mode

        @property
        def command_mode(self):
            return self._js_mapping_mode_labels[self.input_mode.js_mapping_mode - 1]

        @property
        def direction(self):
            if self.input_mode.direction_flag:
                return self._direction_labels[0]
            else:
                return self._direction_labels[1]


    def __init__(self, widget):
        super(InputModeViewerWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('rqt_sensor_viewers'), 'resource', 'InputModeViewer.ui')
        loadUi(ui_file, self)
        self._topic_name = DEFAULT_TOPIC_NAME

        self._subscriber = rospy.Subscriber(self._topic_name, InputMode, self._input_mode_callback)
        self.input_mode = self.ExtendedInputMode()

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    # override
    def start(self):
        self._updateTimer.start(300)  # loop rate[ms]
        self._init_lineEdit()
        self._update_lineEdit()

    def _init_lineEdit(self):
        def init_le(lineEdit):
            lineEdit.clear()
            lineEdit.setReadOnly(True)
            lineEdit.setAlignment(Qt.AlignCenter)

        init_le(self.qt_command_mode_lineEdit)
        init_le(self.qt_direction_lineEdit)

    # override
    def _update_lineEdit(self):
        self.qt_command_mode_lineEdit.setText(self.input_mode.command_mode)
        self.qt_direction_lineEdit.setText(self.input_mode.direction)


    def _input_mode_callback(self, input_mode):
        self.input_mode.set_input_mode(input_mode)
        self._update_lineEdit()

    # override
    def stop(self):
        self._subscriber.unregister()
        self._updateTimer.stop()

    # override
    def timeout_callback(self):
        pass

    # override
    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic_name', self._topic_name)

    # override
    def restore_settings(self, plugin_settings, instance_settings):
        topic_name = instance_settings.value('topic_name')
        try:
            self._topic_name = eval(topic_name)
        except Exception:
            self._topic_name = DEFAULT_TOPIC_NAME

    # override
    def shutdown_plugin(self):
        self.stop()

