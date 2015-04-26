#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_switcher.srv import ImageSwitcherService

DEBUG = True


class ImageListener:
    def __init__(self, node_name, listening_topic_name, received_image):
        self._node_name = node_name
        self._topic_name = listening_topic_name
        self._image_msg = received_image

    def img_callback(self, _img_msg):
        self._image_msg = _img_msg

    def activate(self):
        rospy.Subscriber(self._topic_name, Image, self.img_callback)


class ImageSwitcher:
    def __init__(self):
        # remap-able
        _cameras_list = ['front_cam', 'overview_cam', 'back_cam']  # these will be node_name
        _sub_topics_list = ['front_img', 'overview_img', 'back_img']  # these will be sub_topic_name
        _pub_topics_list = ['display_main', 'display_sub1', 'display_sub2']  # these will be pub_topic_name

        self.main_camera = 1
        self._image_listeners = []
        self._image_publishers = []
        self._pub_img_msgs = [Image()] * len(_pub_topics_list)

        for idx in range(len(_cameras_list)):
            self._image_listeners.append(
                ImageListener(_cameras_list[idx], _sub_topics_list[idx], self._pub_img_msgs[idx]))

        for idx in range(len(_pub_topics_list)):
            self._image_publishers.append(
                rospy.Publisher(_pub_topics_list[idx], Image, queue_size=1))

    def _handle_image_switcher(self, req):
        if DEBUG: print('switch!')
        if req.main_camera_id is not self.main_camera:
            cur_main = self.main_camera - 1
            new_main = req.main_camera_id - 1
            self._pub_img_msgs[cur_main], self._pub_img_msgs[new_main] = self._pub_img_msgs[new_main], \
                                                                         self._pub_img_msgs[cur_main]
            self.main_camera = req.main_camera_id

    def activate(self):
        for image_listener in self._image_listeners:
            image_listener.activate()

        service = rospy.Service('image_switcher', ImageSwitcherService, self._handle_image_switcher)
        print 'READY...'
        # rospy.spin() # spin() is called in ros_looprate_manager.sleep()

    def pub_images(self):
        for idx, image_pub in enumerate(self._image_publishers):
            image_pub.publish(self._pub_img_msgs[idx])


if __name__ == '__main__':
    rospy.init_node('my_image_publisher', anonymous=True)

    image_switcher = ImageSwitcher()
    image_switcher.activate()

    ros_loop_mgr = rospy.Rate(30)  # hz
    while not rospy.is_shutdown():
        image_switcher.pub_images()
        ros_loop_mgr.sleep()

