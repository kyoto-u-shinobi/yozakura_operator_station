#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import division
import os
import numpy as np
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QMenu, QWidget, QImage

import rospy
from rostopic import get_topic_class
from tf.transformations import quaternion_about_axis

from OpenGL.GL import *

from gl_widget import GLWidget as MyGLWidget
from theta_view_events import ThetaViewEvents
import gl_painter as gl_painter

## main class inherits from the ui window class
## hint: http://vivi.dyndns.org/tech/Qt/openGL.html


class ThetaViewWidget(QWidget):
    def __init__(self, plugin):
        super(ThetaViewWidget, self).__init__()
        rp = rospkg.RosPack()
        loadUi(os.path.join(rp.get_path('rqt_theta_viewer'), 'resource', 'ThetaViewerWidget.ui'), self)
        self.plugin = plugin
        self.events = ThetaViewEvents(self)

        self._pushButton_open.clicked.connect(self.events.open_FileDialog)
        self._pushButton_save.clicked.connect(self.events.save_FileDialog)
        self._pushButton_shutter.clicked.connect(self.events.shutter_clicked_event)

        self.initialize_vals()
        self.initialize_glview()
        self.initialize_timer()

    # ==============================================
    # rqt requires
    def save_settings(self, plugin_settings, instance_settings):
        view_matrix_string = repr(self._glview.get_view_matrix())
        instance_settings.set_value('view_matrix', view_matrix_string)

    def restore_settings(self, plugin_settings, instance_settings):
        view_matrix_string = instance_settings.value('view_matrix')
        try:
            view_matrix = eval(view_matrix_string)
        except Exception:
            view_matrix = None

        if view_matrix is not None:
            self._glview.set_view_matrix(view_matrix)
        else:
            self.set_default_view()

    def shutdown_plugin(self):
        pass

    # ==============================================
    # QGLWidget requires
    def set_default_view(self):
        self._glview.makeCurrent()
        self._glview.reset_view()
        self._glview.rotate((0, 0, 1), 45)
        self._glview.rotate((1, 0, 0), -45)
        self._glview.translate((0, 0, -200))

    def update_timeout(self):
        self._glview.makeCurrent()
        self._glview.updateGL()
        self._glview.check_qrcode()
        glRotated(45, 0, 0, 1)

    def glview_paintGL(self):
        self._glview.paintGL_original()

        # draw the axis, the plain and something
        gl_painter.draw_basic_objects()

        self.texture = self._glview.get_texture(self.qimage)
        gl_painter.map_texture_on_sphere(self.texture, self.sphere_radius , 30, 30)

    def glview_mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            menu = QMenu(self._glview)
            action = QAction(self._glview.tr("Reset view"), self._glview)
            menu.addAction(action)
            action.triggered.connect(self.set_default_view)
            menu.exec_(self._glview.mapToGlobal(event.pos()))

    def glview_wheelEvent(self, event):
        # only zoom when no mouse buttons are pressed, to prevent interference with other user interactions
        if event.buttons() == Qt.NoButton:
            d = float(event.delta()) / 200.0 * self._glview._radius
            # TODO: make the moving stop when going to the outside of the sphere
            self._glview.translate([0.0, 0.0, d])
            self._glview.updateGL()
            event.accept()

    # ==============================================
    # initialize
    def initialize_vals(self):
        self.position = (0.0, 0.0, 0.0)
        self.orientation = quaternion_about_axis(0.0, (1.0, 0.0, 0.0))
        self.topicName = None
        self.subscriber = None

        self.sphere_radius = 200
        self.qimage = None
        self.texture = None

        self.filename = None
        self.jpeg_data = None


    def initialize_glview(self):
        # create GL view
        self._glview = MyGLWidget()
        self._glview.setAcceptDrops(True)

        # backup and replace original paint method
        # self.glView.paintGL is callbacked from QGLWidget
        self._glview.paintGL_original = self._glview.paintGL
        self._glview.paintGL = self.glview_paintGL

        # backup and replace original mouse release method
        self._glview.mouseReleaseEvent_original = self._glview.mouseReleaseEvent
        self._glview.mouseReleaseEvent = self.glview_mouseReleaseEvent

        # backup and replace original mouse release method
        self._glview.wheelEvent_original = self._glview.wheelEvent
        self._glview.wheelEvent = self.glview_wheelEvent

        # add GL view to widget layout
        # http://doc.qt.io/qt-4.8/qgridlayout.html
        self.layout().addWidget(self._glview, 1, 0, 1, 4)

        self.qimage = QImage(self.filename, 'JPEG')  # GL_TEXTURE_2D
        # self.qimage = QImage('/home/matsunolab/Pictures/testimage_big.jpg', 'JPEG')  # GL_TEXTURE_2D


    def initialize_timer(self):
        # updateTimeout is called with interval time
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_timeout)
        # init and start update timer with 40ms (25fps)
        self.update_timer.start(40)















