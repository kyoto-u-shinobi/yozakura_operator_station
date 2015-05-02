#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os, time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QSize
from python_qt_binding.QtGui import QTableWidget, QTableWidgetItem, QWidget, QPalette, QBrush, QAbstractItemView, QFont, \
    QHeaderView
import rospkg
import rospy
from rospy.exceptions import ROSException

from yozakura_msgs.msg import HeatSensorData

# エラーとか起こってもとりあえずデータは表示するためのフラグ
DEBUG = False


class HeatSensorViewerWidget(QWidget):
    TOPIC_NAME = 'heat_sensor'
    MAX_COL = 4
    MAX_ROW = 4
    FONT = "Helvetica"
    FONT_SIZE = 15
    FONT_WEIGHT = 20
    DATA_BW_RY = 100
    DATA_BW_YG = 50

    def __init__(self, widget):
        super(HeatSensorViewerWidget, self).__init__()
        rospkg_pack = rospkg.RosPack()
        ui_file = os.path.join(rospkg_pack.get_path('rqt_sensor_viewers'), 'resource', 'HeatSensorViewer.ui')
        loadUi(ui_file, self)

        self._topic_name = self.TOPIC_NAME
        self.topic_edit.setText('/' + self._topic_name)
        self._subscriber = rospy.Subscriber(self._topic_name, HeatSensorData, self._heat_sensor_data_callback)
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
        self._initialize_table(self.table_left, [0.0] * (self.MAX_COL * self.MAX_ROW))
        self._initialize_table(self.table_right, [0.0] * (self.MAX_COL * self.MAX_ROW))

    def _initialize_table(self, table_widget, init_data_list):

        table_widget.clear()
        table_widget.setColumnCount(self.MAX_COL)
        table_widget.setRowCount(self.MAX_ROW)
        table_widget.setSelectionMode(QAbstractItemView.NoSelection)  # セル選択不可
        table_widget.setEditTriggers(QAbstractItemView.NoEditTriggers)  # セル編集不可
        table_widget.horizontalHeader().setResizeMode(QHeaderView.Stretch)
        table_widget.verticalHeader().setResizeMode(QHeaderView.Stretch)

        # 配列を２次元配列にして行列的に扱う
        data_mat = [init_data_list[i * self.MAX_COL: (i + 1) * self.MAX_COL] for i in range(self.MAX_COL)]
        for i, j in [(i, j) for i in range(self.MAX_ROW) for j in range(self.MAX_COL)]:
            item = QTableWidgetItem(str(data_mat[i][j]), Qt.AlignCenter)
            item.setTextAlignment(Qt.AlignCenter)
            item.setFont(QFont(self.FONT, self.FONT_SIZE, self.FONT_WEIGHT, italic=False))
            table_widget.setItem(i, j, item)

        table_widget.resizeColumnsToContents()
        table_widget.resizeRowsToContents()

    def stop(self):
        self._updateTimer.stop()

    def timeout_callback(self):
        if DEBUG:
            if self.data_received_time != None and time.time() - self.data_received_time > 3.0:
                self._update_display_data(self.table_left, False, [None] * (self.MAX_COL * self.MAX_ROW))
                self._update_display_data(self.table_right, False, [None] * (self.MAX_COL * self.MAX_ROW))

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
        new_topic_name = str(self.topic_edit.text())
        if new_topic_name.find('/'):
            new_topic_name = new_topic_name[new_topic_name.find('/') + 1:]

        if new_topic_name is not self._topic_name:
            self._subscriber.unregister()
            self._topic_name = new_topic_name
            self._subscriber = rospy.Subscriber(self._topic_name, HeatSensorData, self._heat_sensor_data_callback)

    def _heat_sensor_data_callback(self, heat_sensor_data):
        self.data_received_time = time.time()
        self._update_display_data(self.table_left,
                                  heat_sensor_data.is_ok,
                                  heat_sensor_data.data[0:16])
        self._update_display_data(self.table_right,
                                  heat_sensor_data.is_ok,
                                  heat_sensor_data.data[16:32])

    def _update_display_data(self, table_widget, is_ok, data_list):
        def _set_data(_is_ok, _data_mat):
            for i, j in [(i, j) for i in range(self.MAX_ROW) for j in range(self.MAX_COL)]:
                item = table_widget.item(i, j)
                if not _is_ok:
                    item.setBackground(self._white_palette.brush(QPalette.Base))
                else:
                    item.setText(str(_data_mat[i][j]))
                    if data_mat[i][j] < self.DATA_BW_YG:
                        item.setBackground(self._green_palette.brush(QPalette.Base))
                    elif self.DATA_BW_YG <= data_mat[i][j] < self.DATA_BW_RY:
                        item.setBackground(self._yellow_palette.brush(QPalette.Base))
                    else:
                        item.setBackground(self._red_palette.brush(QPalette.Base))

        # debugでなければ常にデータ表示する
        if not DEBUG:
            is_ok = True

        if is_ok is not True:
            self.topic_edit.setPalette(self._red_palette)
            data_mat = [[None] * self.MAX_COL] * self.MAX_ROW
            _set_data(False, data_mat)
        else:
            self.topic_edit.setPalette(self._green_palette)
            data_mat = [data_list[i * self.MAX_COL: (i + 1) * self.MAX_COL] for i in range(self.MAX_COL)]
            _set_data(True, data_mat)

