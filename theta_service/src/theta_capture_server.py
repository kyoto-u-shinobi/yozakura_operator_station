#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import CompressedImage
from theta_service.srv import ThetaCaptureService, ThetaCaptureServiceResponse
from theta.theta import Theta

DEBUG = True
RESIZED_IMAGE_FLAG = True


class Theta360Server:
    GRAB_CURRENT_IMAGE = 1

    def __init__(self):
        self.theta = Theta()
        self.resized_image_flag = rospy.get_param('~is_image_resized', RESIZED_IMAGE_FLAG)

    def initialize(self):
        if self.theta.open():
            self.theta.set_init_settings()
            self.service = rospy.Service('theta_capture', ThetaCaptureService, self.handle_theta_capture)
            print 'READY...'
            return True
        else:
            return False

    def generate_imgmsg(self, image):
        jpeg_img = CompressedImage()
        # data set
        jpeg_img.header.stamp = rospy.Time.now()
        jpeg_img.format = "jpeg"
        jpeg_img.data = image
        return jpeg_img

    def handle_theta_capture(self, req):
        if req.capture_mode == self.GRAB_CURRENT_IMAGE:
            self.theta.shutter()
            image = self.theta.grab_currentest_image(RESIZED_IMAGE_FLAG)
            print 'grab image!'
            return ThetaCaptureServiceResponse(self.generate_imgmsg(image))
        else:
            print 'grab image!'
            image = self.theta.grab_currentest_image(RESIZED_IMAGE_FLAG)
            return ThetaCaptureServiceResponse(self.generate_imgmsg(image))


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('theta_capture_server')
    theta_server = Theta360Server()
    if theta_server.initialize() or DEBUG:
        rospy.spin()
    else:
        print('fail to initialize theta')
