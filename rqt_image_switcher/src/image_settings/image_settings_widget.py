#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os, time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QWidget
import rospkg
import rospy
from rospy.exceptions import ROSException

from web_cam.srv import AIballSettings, AIballSettingsResponse

# remap-able
DEFAULT_SERVICE_NAME_FRONT = 'front_cam_settings'
DEFAULT_SERVICE_NAME_BACK = 'back_cam_settings'
DEFAULT_SERVICE_NAME_OVERVIEW = 'overview_cam_settings'
DEFAULT_SERVICE_NAME_ARM = 'arm_cam_settings'

# param-able
DEFAULT_RESOLUTION = 'VGA'
DEFAULT_COMPRESS_MODE = 0


class ImageSettingsWidget(QWidget):
    class AIBallSettings:
        def __init__(self):
            self.resolution = rospy.get_param('~resolution', DEFAULT_RESOLUTION)
            self.compress_mode = rospy.get_param('~compress_mode', DEFAULT_COMPRESS_MODE)
            self.contrast = 0
            self.brightness = 0

        @property
        def settings(self):
            return self.resolution, self.compress_mode, self.contrast, self.brightness

    def __init__(self, widget):
        super(ImageSettingsWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('rqt_image_switcher'), 'resource', 'ImageSettings.ui')
        loadUi(ui_file, self)

        self._service_names = [DEFAULT_SERVICE_NAME_FRONT,
                               DEFAULT_SERVICE_NAME_BACK,
                               DEFAULT_SERVICE_NAME_OVERVIEW,
                               DEFAULT_SERVICE_NAME_ARM]
        self._selected_service_name = self._service_names[0]

        self._services, self._settings = [], []
        for name in self._service_names:
            self._services.append(rospy.ServiceProxy(name, AIballSettings))
            self._settings.append(self.AIBallSettings())

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    def start(self):
        self._updateTimer.start(0.1)  # loop rate [ms]
        self.qt_services_combo.addItems(self._service_names)
        # for idx in range(len(self._services)):
        # self._call_service(idx)

    def stop(self):
        self._updateTimer.stop()

    def timeout_callback(self):
        self._selected_service_name = self.qt_services_combo.currentText()

    # rqt override
    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('selected_service', self._selected_service_name)

    def restore_settings(self, plugin_settings, instance_settings):
        selected_service = instance_settings.value('selected_service')
        try:
            self.selected_service = eval(selected_service)
        except Exception:
            self.selected_service = DEFAULT_SERVICE_NAME_FRONT

    def shutdown_plugin(self):
        self.stop()

    def _call_service(self, idx):
        resolution, compress_mode, contrast, brightness = self._settings[idx].settings
        self._services[idx](resolution, compress_mode, contrast, brightness)

    @Slot()
    def on_qt_vga_btn_clicked(self):
        idx = self._service_names.index(self._selected_service_name)
        if self._settings[idx].resolution is not 'VGA':
            self._settings[idx].resolution = 'VGA'
            self._call_service(idx)

    @Slot()
    def on_qt_qvga_btn_clicked(self):
        idx = self._service_names.index(self._selected_service_name)
        if self._settings[idx].resolution is not 'QVGA':
            self._settings[idx].resolution = 'QVGA'
            self._call_service(idx)

    @Slot()
    def on_qt_contrast_plus_btn_clicked(self):
        idx = self._service_names.index(self._selected_service_name)
        self._settings[idx].contrast = 1
        self._call_service(idx)
        self._settings[idx].contrast = 0

    @Slot()
    def on_qt_contrast_minus_btn_clicked(self):
        idx = self._service_names.index(self._selected_service_name)
        self._settings[idx].contrast = -1
        self._call_service(idx)
        self._settings[idx].contrast = 0

    @Slot()
    def on_qt_brightness_plus_btn_clicked(self):
        idx = self._service_names.index(self._selected_service_name)
        self._settings[idx].brightness = 1
        self._call_service(idx)
        self._settings[idx].brightness = 0

    @Slot()
    def on_qt_brightness_minus_btn_clicked(self):
        idx = self._service_names.index(self._selected_service_name)
        self._settings[idx].brightness = -1
        self._call_service(idx)
        self._settings[idx].brightness = 0

