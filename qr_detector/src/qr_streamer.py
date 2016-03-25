#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Streamer(object):
    def __init__(self, stream_address):
        self.cap = cv2.VideoCapture(stream_address)
        self.bridge = CvBridge()
        self.camera_pub = rospy.Publisher('camera_image', Image, queue_size = 10)
    def GetLatestAndPublish(self):
        _, self.frame = self.cap.read()
        self.image = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
        self.camera_pub.publish(self.image)
        
if __name__ == '__main__':
    rospy.init_node('qr_streamer')
    rate = rospy.Rate(50)
    streamer = Streamer('rtsp://192.168.54.150/stream1')
    while not rospy.is_shutdown():
        streamer.GetLatestAndPublish()
        rate.sleep()

