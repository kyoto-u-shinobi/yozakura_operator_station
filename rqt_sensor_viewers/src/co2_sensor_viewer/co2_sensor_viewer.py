#!/usr/bin/env python
# -*- coding: utf-8 -*-


from rqt_gui_py.plugin import Plugin
from .co2_sensor_viewer_widget import CO2SensorViewerWidget


class CO2SensorViewer(Plugin):
    def __init__(self, context):
        super(CO2SensorViewer, self).__init__(context)
        self.setObjectName('CO2SensorViewer')
        self._widget = CO2SensorViewerWidget(self)
        self._widget.start()
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
