#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QTreeWidget, QTreeWidgetItem, QWidget, QPalette, QBrush
import rospkg
import rospy
from rospy.exceptions import ROSException

from yozakura_msgs.msg import HeatSensorData


class HeatSensorViewerWidget(QWidget):
    TOPIC_NAME = 'heat_sensor'
    DATA_BW_RY = 100
    DATA_BW_YG = 50

    def __init__(self, widget):
        super(HeatSensorViewerWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('heat_sensor_viewer'), 'resource', 'HeatSensorViewer.ui')
        loadUi(ui_file, self)

        self._topic_name = self.TOPIC_NAME
        self.topic_edit.setText('/' + self._topic_name)
        self._subscriber = rospy.Subscriber(self._topic_name, HeatSensorData, self._heat_sensor_data_callback)

        self._red_palette = QPalette()
        self._red_palette.setColor(QPalette.Base, Qt.red)
        self._yellow_palette = QPalette()
        self._yellow_palette.setColor(QPalette.Base, Qt.yellow)
        self._green_palette = QPalette()
        self._green_palette.setColor(QPalette.Base, Qt.green)

        self._updateTimer = QTimer(self)
        self._updateTimer.timeout.connect(self.timeout_callback)

    def start(self):
        def _set_items(tree_widget, data_list):
            tree_widget.clear()
            for idx, data in enumerate(data_list):
                item = QTreeWidgetItem(tree_widget, str(data), QTreeWidgetItem.DontShowIndicator)
                item.setTextAlignment(0, Qt.AlignCenter)
                tree_widget.insertTopLevelItem(0, item)

        self._updateTimer.start(1000)
        _set_items(self.tree_column_left, [0.0] * 16)
        _set_items(self.tree_column_right, [0.0] * 16)


    def stop(self):
        self._updateTimer.stop()

    def timeout_callback(self):
        pass

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
            self._topic_name = str(self.topic_edit.text())[1:]
            self._subscriber = rospy.Subscriber(self._topic_name, HeatSensorData, self._heat_sensor_data_callback)

    def _heat_sensor_data_callback(self, heat_sensor_data):
        self._update_display_data(heat_sensor_data.is_ok,
                                  heat_sensor_data.data[0:16],
                                  heat_sensor_data.data[16:32])

    def _update_display_data(self, is_ok, data_list_left, data_list_right):
        def _set_items(tree_widget, data_list):
            for idx, data in enumerate(data_list):
                item = tree_widget.topLevelItem(idx)
                item.setText(0, str(data))
                if data > self.DATA_BW_RY:
                    item.setBackground(0, self._red_palette.brush(QPalette.Base))
                elif data < self.DATA_BW_YG:
                    item.setBackground(0, self._green_palette.brush(QPalette.Base))
                else:
                    item.setBackground(0, self._yellow_palette.brush(QPalette.Base))

        if is_ok is not True:
            self.topic_edit.setPalette(self._red_palette)
        else:
            self.topic_edit.setPalette(self._green_palette)
            _set_items(self.tree_column_left, data_list_left)
            _set_items(self.tree_column_right, data_list_right)























