#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os, time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QSize
from python_qt_binding.QtGui import QWidget, QPalette, QFont
import rospkg
import rospy
from rospy.exceptions import ROSException

from yozakura_msgs.msg import CO2SensorData


class CO2SensorViewerWidget(QWidget):
    TOPIC_NAME = 'co2_sensor'
    FONT = "Helvetica"
    FONT_SIZE = 15
    FONT_WEIGHT = 20
    DATA_BW_RY = 1000
    DATA_BW_YG = 750

    def __init__(self, widget):
        super(CO2SensorViewerWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('rqt_sensor_viewers'), 'resource', 'CO2SensorViewer.ui')
        loadUi(ui_file, self)

        self._topic_name = self.TOPIC_NAME
        self.topic_edit.setText('/' + self._topic_name)
        self._subscriber = rospy.Subscriber(self._topic_name, CO2SensorData, self._co2_sensor_data_callback)
        self.data_received_time = None

        self._white_palette = QPalette()
        self._white_palette.setColor(QPalette.Base, Qt.white)
        self._red_palette = QPalette()
        self._red_palette.setColor(QPalette.Base, Qt.red)
        self._yellow_palette = QPalette()
        self._yellow_palette.setColor(QPalette.Base, Qt.yellow)
        self._green_palette = QPalette()
        self._green_palette.setColor(QPalette.Base, Qt.green)

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    def start(self):
        self._updateTimer.start(1000)  # loop rate is 1000[ms]
        self._initialize_line_edit(self.data_edit, 0.0)

    def _initialize_line_edit(self, line_edit, init_data):

        line_edit.clear()
        line_edit.setReadOnly(True)
        line_edit.setAlignment(Qt.AlignCenter)
        line_edit.setFont(QFont(self.FONT, self.FONT_SIZE, self.FONT_WEIGHT, italic=False))

        line_edit.setText(str(init_data))


    def stop(self):
        self._updateTimer.stop()

    def timeout_callback(self):
        if self.data_received_time != None and time.time() - self.data_received_time > 3.0:
            self._update_display_data(self.data_edit, False, None)

    # rqt override
    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic_name', self._topic_name)

    def restore_settings(self, plugin_settings, instance_settings):
        topic_name = instance_settings.value('topic_name')
        try:
            self._topic_name = eval(topic_name)
        except Exception:
            self._topic_name = self.TOPIC_NAME

    def shutdown_plugin(self):
        self.stop()

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        if str(self.topic_edit.text()) is not self._topic_name:
            self._subscriber.unregister()
            if str(self.topic_edit.text()).find('/'):
                self._topic_name = str(self.topic_edit.text())[str(self.topic_edit.text()).find('/') + 1:]
            self._subscriber = rospy.Subscriber(self._topic_name, CO2SensorData, self._co2_sensor_data_callback)

    def _co2_sensor_data_callback(self, co2_sensor_data):
        self.data_received_time = time.time()
        self._update_display_data(self.data_edit,
                                  co2_sensor_data.is_ok,
                                  co2_sensor_data.data)

    def _update_display_data(self, line_edit, is_ok, data):
        if is_ok is not True:
            self.topic_edit.setPalette(self._red_palette)
            line_edit.setPalette(self._white_palette)
        else:
            self.topic_edit.setPalette(self._green_palette)
            line_edit.setText(str(data))
            if data < self.DATA_BW_YG:
                line_edit.setPalette(self._green_palette)
            elif self.DATA_BW_YG <= data < self.DATA_BW_RY:
                line_edit.setPalette(self._yellow_palette)
            else:
                line_edit.setPalette(self._red_palette)


