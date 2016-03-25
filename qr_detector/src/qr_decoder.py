#!/usr/bin/env python

import rospy
import cv2
import zbar
import Image as Img
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Decoder(object):
    def __init__(self):
        self.bridge = CvBridge()
        self._camera_sub = rospy.Subscriber('camera_image', Image, self.camera_callback)
        self._detect_pub = rospy.Publisher('detected_image', Image, queue_size=10)
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')
    def camera_callback(self, imgmsg):
        frame = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pil = Img.fromarray(gray)
        width, height = pil.size
        raw = pil.tostring()
        img = zbar.Image(width, height, 'Y800', raw)
        self.scanner.scan(img)
        symbolPos = []
        disp_img = imgmsg
        for symbol in img:
            symbolPos = symbol.location
            string = '[%s]' % symbol.data
            rospy.loginfo(string)
            cv2.line(frame, symbolPos[0], symbolPos[1], (0, 0, 255), 3)
            cv2.line(frame, symbolPos[0], symbolPos[3], (0, 0, 255), 3)
            cv2.line(frame, symbolPos[2], symbolPos[1], (0, 0, 255), 3)
            cv2.line(frame, symbolPos[2], symbolPos[3], (0, 0, 255), 3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, string, symbolPos[0], font, 1, (0, 127, 255), 2)
            disp_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self._detect_pub.publish(disp_img)

if __name__ == '__main__':
    rospy.init_node('qr_decoder')
    decoder = Decoder()
    rospy.spin()

