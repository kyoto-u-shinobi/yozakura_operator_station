#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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


class WebCamManager:
    def __init__(self, ip_address, topic_name, overlayed_text=None):
        self.cvbridge = CvBridge()
        self.capture = cv2.VideoCapture('http://' + ip_address + '/?action=stream.mjpeg')
        self.topic_name = topic_name
        self.overlayed_text = overlayed_text
        print self.overlayed_text
        self.is_active = False

    def open(self):
        return self.capture.isOpened()

    def activate(self):
        self.is_active = True
        self.pub_image = rospy.Publisher(self.topic_name, Image, queue_size=1)

    def publish_img(self):
        if not self.is_active:
            self.activate()

        has_image, cv_image = self.capture.read()
        h, w = cv_image.shape[0], cv_image.shape[1]

        if has_image is False:
            print('fail to grub image')
            text = '!! FAIL TO GRUB !!'
            text_pxlength = 13.0 * len(text)
            cv2.putText(cv_image, text,
                        (int(w / 2.0 - text_pxlength / 2.0), int(h / 2.0)),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=30, color=(255, 0, 0), thickness=20)

        if self.overlayed_text is not None:
            text_pxlength = 13.0 * len(self.overlayed_text)
            cv2.putText(cv_image, self.overlayed_text,
                        (int(w / 2.0 - text_pxlength / 2.0), int(h / 16.0)),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 0, 255), thickness=2)

        try:
            self.pub_image.publish(self.cvbridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print e

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(30)  # Hz
    web_cam_ip = rospy.get_param('~ip_address', DEFAULT_IP_ADDRESS)
    overlay_text = rospy.get_param('~overlay_text', DEFAULT_IP_ADDRESS)

    web_cam = WebCamManager(web_cam_ip, DEFAULT_TOPIC_NAME, overlay_text)

    if not web_cam.open():
        print('fail to open!')
    else:
        if (DEBUG): print('run!')
        web_cam.activate()
        while not rospy.is_shutdown():
            web_cam.publish_img()
            rate_mgr.sleep()



