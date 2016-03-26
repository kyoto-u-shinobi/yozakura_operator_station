from math import sqrt, sin, cos, tan, atan, pi
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LiveView(object):
    def __init__(self, stream='rtsp://192.168.54.150/stream1'):
        self.valid_ratio_x = 0.88
        self.valid_ratio_y = 0.88
        self._sub_image = rospy.Subscriber('camera_image', Image, self.image_callback)
        # self.capture = cv2.VideoCapture(stream)
        # self.capture = cv2.VideoCapture(1)
        self.bridge = CvBridge()
        self.UpdateMap()

    def image_callback(self, img):
        self.frame = self.bridge.imgmsg_to_cv2(img)

    def UpdateMap(self):
        """
        Generate map data between the video stream and the texture.

        Call this function when the resolution has been changed.
        """
        # _, frame = self.capture.read()
        # self.rows, self.cols = frame.shape[:2]
        self.rows, self.cols = 1080, 1920
        self.map_x = np.zeros((self.cols / 2, self.cols), np.float32)
        self.map_y = np.zeros((self.cols / 2, self.cols), np.float32)
        square_side = self.cols / 2
        r = square_side / 2
        for j in xrange(square_side):
            for i in xrange(square_side):
                # left sphere
                u = 1.0 * (i - r) * pi / (2 * r)
                v = 1.0 * (j - r) * pi / (2 * r)
                x, y = self._Equirectangular2Fisheye(u, v)
                self.map_x.itemset((j, i), r * y * self.valid_ratio_y + r)
                self.map_y.itemset((j, i), r * -x * self.valid_ratio_x + r)
                # right sphere
                i = i + square_side
                u = 1.0 * (i - r - square_side) * pi / (2 * r)
                v = 1.0 * (j - r) * pi / (2 * r)
                x, y = self._Equirectangular2Fisheye(u, v)
                self.map_x.itemset((j, i), r * -y * self.valid_ratio_y + r + square_side)
                self.map_y.itemset((j, i), r * x * self.valid_ratio_x + r)

    def _Equirectangular2Fisheye(self, u, v):
        """
        Convert equirectangular coordinates (longitude and latitude) into fisheye coordinates.
        """
        if u == 0.0 and v == 0.0:
            return 0, 0
        elif v == 0.0:
            return sin(u), 0
        else:
            u2 = tan(u / 2)
            v2 = tan(v / 2)
            x = -cos(2.0 * atan(((cos(u) + 1) * (u2 + sqrt((u2 ** 2 + v2 ** 2)/(u2 ** 2 * v2 ** 2 + 1)) - u2 * v2 ** 2 + u2 ** 2 * v2 ** 2 * sqrt((u2 ** 2 + v2 ** 2)/(u2 ** 2 * v2 ** 2 + 1))))/(2.0 * v2))) * sin(2.0 * atan(sqrt((u2 ** 2 + v2 ** 2)/(u2 ** 2 * v2 ** 2 + 1))))
            y = sin(2.0 * atan(((cos(u) + 1) * (u2 + sqrt((u2 ** 2 + v2 ** 2)/(u2 ** 2 * v2 ** 2 + 1)) - u2 * v2 ** 2 + u2 ** 2 * v2 ** 2 * sqrt((u2 ** 2 + v2 ** 2)/(u2 ** 2 * v2 ** 2 + 1))))/(2.0 * v2))) * sin(2.0 * atan(sqrt((u2 ** 2 + v2 ** 2)/(u2 ** 2 * v2 ** 2 + 1))))
            return x, y

    def GetNewRemappedFrame(self):
        # _, self.frame = self.capture.read()
        self.frame_remapped = cv2.remap(self.frame, self.map_x, self.map_y, cv2.INTER_AREA)
        return self.frame_remapped
