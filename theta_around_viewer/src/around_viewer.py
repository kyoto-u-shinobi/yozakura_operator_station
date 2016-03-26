#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
from math import sqrt, sin, cos, atan2, pi
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

DEFAULT_STREAM_ADDRESS = "rtsp://172.16.1.1/stream1"

class AroundViewer(object):
    def __init__(self):
        self.valid_ratio_x = 0.88
        self.valid_ratio_y = 0.88
        self.width = 800
        self.height = 800
        # self.capture = cv2.VideoCapture(stream)
        # self.capture = cv2.VideoCapture(1)
        # self.capture = cv2.VideoCapture('http://10.249.255.137:8088')
        self.UpdateMap()
        self.frame = []

        self._sub_image = rospy.Subscriber('camera_image', Image, self.image_callback)
        self._pub_image = rospy.Publisher('around_image', Image, queue_size = 10)
        self._bridge = CvBridge()

    def image_callback(self, img):
        self.frame = self._bridge.imgmsg_to_cv2(img)

    def UpdateMap(self):
        """
        Generate map data between video stream and output

        Call this function when the resolution has been changed.
        """
        # _, frame = self.capture.read()
        # self.rows, self.cols = frame.shape[:2]
        self.rows, self.cols = 1080, 1920
        self.map_x = np.zeros((self.width, self.height), np.float32)
        self.map_y = np.zeros((self.width, self.height), np.float32)
        for j in xrange(self.width):
            for i in xrange(self.height):
                u = 1.0 * (j - self.width / 2) * pi * 2 / sqrt(2) / self.width
                v = 1.0 * (i - self.height / 2) * pi * 2 / sqrt(2) / self.height
                x, y = self._LittlePlanet2Fisheye(u, v, self.cols/4)
                self.map_x.itemset((j, i), x)
                self.map_y.itemset((j, i), y)

    def _LittlePlanet2Fisheye(self, u, v, r):
        if u == 0.0:
            if v == 0.0:
                phi = 0.0
                theta = 0.0
            elif v > 0.0:
                phi = pi / 2
                theta = v
            else:
                phi = pi / 2 * 3
                theta = -v
        else:
            phi = atan2(v, u)
            theta = sqrt(u ** 2 + v ** 2)
        x_0 = sin(theta) * cos(phi)
        y_0 = sin(theta) * sin(phi)
        z_0 = cos(theta)
        x, y, z = self._Rotate3D(x_0, y_0, z_0)
        if x < 0:
            ret_y = -r * z * self.valid_ratio_x + r 
            ret_x = r * y * self.valid_ratio_y + r
        else:
            ret_y = -r * z * self.valid_ratio_x + r
            ret_x = -r * y * self.valid_ratio_y + 3 * r
        return ret_x, ret_y

    def _Rotate3D(self, x_0, y_0, z_0):
        x = y_0
        y = x_0
        z = z_0
        return x, y, z

    def GetNewRemappedFrame(self):
        # _, self.frame = self.capture.read()
        if self.frame == []:
            time.sleep(1)
        self.frame_remapped = cv2.remap(self.frame, self.map_x, self.map_y, cv2.INTER_AREA)
        return self.frame_remapped

    def PublishImage(self):
        self.image = self._bridge.cv2_to_imgmsg(self.frame_remapped, "bgr8")
        self._pub_image.publish(self.image)
        
if __name__ == '__main__':
    rospy.init_node('around_viewer')
#    stream_address = rospy.get_param("stream_address", DEFAULT_STREAM_ADDRESS)
#    stream_address = 'rtsp://192.168.54.150/stream1'
    rate_mgr = rospy.Rate(50)
#    around_viewer = AroundViewer(stream_address)
    around_viewer = AroundViewer()
    while not rospy.is_shutdown():
        around_viewer.GetNewRemappedFrame()
        around_viewer.PublishImage()
        rate_mgr.sleep()
