#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_switcher.srv import *

DEBUG = True


class ImageListener:
    def __init__(self, _nodename, _listening_topicname, _order, _image_container_list):
        self.nodename = _nodename
        self.topicname = _listening_topicname
        self.image_container_list = _image_container_list
        self.order = _order

    def set_order(self, _order):
        self.order = _order

    def img_callback(self, _imgmsg):
        self.image_container_list[self.order] = _imgmsg

    def run(self):
        rospy.Subscriber(self.topicname, Image, self.img_callback)


class ImageSwitcher:
    def __init__(self):
        self.num_of_cams = 4
        self.switching_mode = 1
        self.switching_order = range(self.num_of_cams)
        self.display_imgs = [Image()] * self.num_of_cams

        self.front_cam_imglistener = ImageListener('front_cam', 'front_cam_img', 0, self.display_imgs)
        self.front_cam_imglistener.run()
        self.overview_cam_imglistener = ImageListener('overview_cam', 'overview_cam_img', 1, self.display_imgs)
        self.overview_cam_imglistener.run()
        self.back_cam_imglistener = ImageListener('back_cam', 'back_cam_img', 2, self.display_imgs)
        self.back_cam_imglistener.run()
        self.arm_cam_imglistener = ImageListener('arm_cam', 'arm_cam_img', 3, self.display_imgs)
        self.arm_cam_imglistener.run()

        self.set_cimage_msg()

    def handle_image_switcher(self, _req):
        if DEBUG: print 'switch!'
        if _req.switching_mode == 1:
            self.switching_order = [0, 1, 2, 3]
        elif _req.switching_mode == 2:
            self.switching_order = [1, 2, 3, 0]
        elif _req.switching_mode == 3:
            self.switching_order = [2, 3, 0, 1]
        elif _req.switching_mode == 4:
            self.switching_order = [3, 0, 1, 2]
        self.set_cimage_msg()

    def set_cimage_msg(self):
        self.front_cam_imglistener.set_order(self.switching_order[0])
        self.overview_cam_imglistener.set_order(self.switching_order[1])
        self.back_cam_imglistener.set_order(self.switching_order[2])
        self.arm_cam_imglistener.set_order(self.switching_order[3])

    def run(self):
        self.set_cimage_msg()
        service = rospy.Service('image_switcher', ImageSwitcherService, self.handle_image_switcher)
        print 'READY...'
        # rospy.spin() # spin() is called in ros_looprate_manager.sleep()


if __name__ == '__main__':
    rospy.init_node('my_image_publisher', anonymous=True)
    img_publishers = []

    image_switcher = ImageSwitcher()
    image_switcher.run()
    try:
        for i in range(image_switcher.num_of_cams):
            img_publishers.append(rospy.Publisher('display' + str(i + 1), Image, queue_size=10))

        ros_looprate_manager = rospy.Rate(30)  # hz
        while not rospy.is_shutdown():
            for i in range(image_switcher.num_of_cams):
                img_publishers[i].publish(image_switcher.display_imgs[i])
            ros_looprate_manager.sleep()

    except rospy.ROSInterruptException:
        print 'Error'

