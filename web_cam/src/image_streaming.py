#!/usr/bin/env python
# -*- coding: utf-8 -*-

import urllib2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from web_cam.srv import AIballSettings, AIballSettingsResponse

import cv2

'''
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
http://atagan-memo.blogspot.jp/2012/11/opencvai-ballwifi.html
http://venuschjp.blogspot.jp/2015/02/pythonopencvweb.html
'''

DEBUG = True
DEFAULT_IP_ADDRESS = '192.168.2.1'
DEFAULT_NODE_NAME = 'web_cam'
DEFAULT_TOPIC_NAME = 'web_cam_img'
DEFAULT_SERVICE_NAME = 'web_cam_settings_service'


class WebCamManager(object):
    class AIballSettings(object):
        def __init__(self, default_resolution, default_compress_modes):
            # key: service, value: postのcommand
            self.resolution = default_resolution
            self.resolutions = {'VGA': 'VGA640_480', 'QVGA': 'QVGA320_240', 'QQVGA': 'QQVGA160_120'}
            self.compress_mode = default_compress_modes
            self.compress_modes = {0: 'standard_compress', 1: 'high_compress'}
            self.contrast = {1: 'contrast_plus', - 1: 'contrast_minus'}
            self.brightness = {1: 'brightness_plus', - 1: 'brightness_minus'}

        def get_resolution_cmd(self):
            return self.resolutions[self.resolution]

        def get_compress_mode_cmd(self):
            return self.compress_modes[self.compress_mode]

        def get_contrast_cmd(self, val):
            return self.contrast[val] if abs(val) is 1 else None

        def get_brightness_cmd(self, val):
            return self.brightness[val] if abs(val) is 1 else None

    def __init__(self, ip_address, topic_name, overlayed_text=None):
        self._cvbridge = CvBridge()
        self._capture = cv2.VideoCapture('http://' + ip_address + '/?action=stream.mjpeg')
        # self._capture = cv2.VideoCapture('http://' + ip_address + '/?action=snapshot.jpeg')
        self._cmd_uri = 'http://' + ip_address + '/?action=command&command='
        self._topic_name = topic_name
        self._overlayed_text = overlayed_text

        self._aiball_setting = self.AIballSettings('QVGA', 0)
        self._send_command(self._aiball_setting.get_resolution_cmd())
        self._send_command(self._aiball_setting.get_compress_mode_cmd())

        self.is_active = False

    def open(self):
        return self._capture.isOpened()

    def activate(self):
        self.is_active = True
        self.service = rospy.Service(DEFAULT_SERVICE_NAME, AIballSettings, self._handle_aiball_settings)
        self.pub_image = rospy.Publisher(self._topic_name, Image, queue_size=1)

    def _send_command(self, cmd):
        urllib2.urlopen(urllib2.Request(self._cmd_uri + cmd))

    def _handle_aiball_settings(self, req):
        if req.resolution is not self._aiball_setting.resolution:
            self._aiball_setting.resolution = req.resolution
            self._send_command(self._aiball_setting.get_resolution_cmd())

        if req.compress_mode is not self._aiball_setting.compress_mode:
            self._aiball_setting.compress_mode = req.compress_mode
            self._send_command(self._aiball_setting.get_compress_mode_cmd())

        if abs(req.contrast) is 1:
            self._send_command(self._aiball_setting.get_contrast_cmd(req.contrast))

        if abs(req.brightness) is 1:
            self._send_command(self._aiball_setting.get_brightness_cmd(req.brightness))

    def publish_img(self):
        if not self.is_active:
            self.activate()

        has_image, cv_image = self._capture.read()
        h, w = cv_image.shape[0], cv_image.shape[1]

        # cv2.putTextのテキスト位置とかは適当に決めた
        if has_image is False:
            print('fail to grub image')
            text = '!! FAIL TO GRUB !!'
            text_pxlength = 13.0 * len(text)
            cv2.putText(cv_image, text,
                        (int(w / 2.0 - text_pxlength / 2.0), int(h / 2.0)),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=30, color=(255, 0, 0), thickness=20)

        if self._overlayed_text is not None:
            text_pxlength = 13.0 * len(self._overlayed_text)
            cv2.putText(cv_image, self._overlayed_text,
                        (int(w / 2.0 - text_pxlength / 2.0), int(h / 8.0)),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 0, 255), thickness=2)

        try:
            self.pub_image.publish(self._cvbridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print e

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz
    web_cam_ip = rospy.get_param('~ip_address', DEFAULT_IP_ADDRESS)
    overlay_text = rospy.get_param('~overlay_text', DEFAULT_IP_ADDRESS)

    web_cam = WebCamManager(web_cam_ip, DEFAULT_TOPIC_NAME, overlay_text)
    print(overlay_text)

    if not web_cam.open():
        print('fail to open!')
    else:
        if (DEBUG): print('run!')
        web_cam.activate()
        while not rospy.is_shutdown():
            web_cam.publish_img()
            rate_mgr.sleep()



