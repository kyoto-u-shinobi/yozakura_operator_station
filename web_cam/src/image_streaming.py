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

AI_BALL_IP_ADDRESS = '192.168.2.1'


class WebCamManager:
    def __init__(self, _ipaddress, _nodename, _topicname):
        self.cvbridge = CvBridge()
        self.capture = cv2.VideoCapture('http://' + _ipaddress + '/?action=stream.mjpeg')
        self.nodename = _nodename
        self.topicname = _topicname

    def open(self):
        return self.capture.isOpened()

    def run(self):
        rospy.init_node(self.nodename, anonymous=True)
        pub_ros_image = rospy.Publisher(self.topicname, Image)
        ros_looprate_manager = rospy.Rate(30)  # hz

        while not rospy.is_shutdown():
            has_image, cv_image = self.capture.read()
            if has_image == False:
                continue
            try:
                pub_ros_image.publish(self.cvbridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError, e:
                print e
            ros_looprate_manager.sleep()


if __name__ == '__main__':
    web_cam_back = WebCamManager(AI_BALL_IP_ADDRESS, 'ai_ball', 'ai_ball_img')
    if DEBUG: print 'initialize!'
    if web_cam_back.open():
        if DEBUG: print 'run!'
        web_cam_back.run()
    else:
        print 'fail to open!'