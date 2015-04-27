#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os, time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QSize
from python_qt_binding.QtGui import QTableWidget, QTableWidgetItem, QWidget, QPalette, QBrush, QAbstractItemView, QFont
import rospkg
import rospy
from rospy.exceptions import ROSException

from image_switcher_node import ImageSwitcher
from rqt_image_switcher.srv import ImageSwitcherService


class ImageSwitchWidget(QWidget):
    def __init__(self, widget):
        super(ImageSwitchWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('rqt_image_switcher'), 'resource', 'ImageSwitch.ui')
        loadUi(ui_file, self)

        self._image_switcher_server = ImageSwitcher()
        self._service_proxy = rospy.ServiceProxy('image_switcher', ImageSwitcherService)

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    def start(self):
        self._updateTimer.start(0.1)  # loop rate is 1000[ms]
        self._image_switcher_server.activate()

    def stop(self):
        self._image_switcher_server.deactivate()
        self._updateTimer.stop()

    def timeout_callback(self):
        self._image_switcher_server.pub_images()

    # rqt override
    def save_settings(self, plugin_settings, instance_settings):
        pass
        # instance_settings.set_value('topic_name', self._topic_name)

    def restore_settings(self, plugin_settings, instance_settings):
        pass
        # topic_name = instance_settings.value('topic_name')
        # try:
        # self._topic_name = eval(topic_name)
        # except Exception:
        # self._topic_name = self.TOPIC_NAME

    def shutdown_plugin(self):
        self.stop()

    @Slot()
    def on_front_cam_btn_clicked(self):
        self._service_proxy(1)

    @Slot()
    def on_back_cam_btn_clicked(self):
        self._service_proxy(2)

    @Slot()
    def on_overview_cam_btn_clicked(self):
        self._service_proxy(3)

