#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_switcher.srv import ImageSwitcherService

DEBUG = True


class ImageListener:
    def __init__(self, node_name, listening_topic_name, received_images, id, order):
        self._node_name = node_name
        self._topic_name = listening_topic_name
        self._image_msgs = received_images
        self.id = id
        self.order = order
        self.is_activate = False

    def set_order(self, order):
        self.order = order

    def img_callback(self, _img_msg):
        self._image_msgs[self.order] = _img_msg

    def activate(self):
        self.is_activate = True
        self._sub = rospy.Subscriber(self._topic_name, Image, self.img_callback)

    def deactivate(self):
        if self.is_activate:
            self._sub.unregister()


class ImageSwitcher:
    def __init__(self):
        # remap-able
        # see ImageSwitcherService.srv for the order of the list
        _camera_names = ['front_cam', 'back_cam', 'overview_cam', 'arm_cam']
        _sub_topic_names = ['front_img', 'back_img', 'overview_img', 'arm_img']
        _pub_topic_names = ['display_main', 'display_sub1', 'display_sub2', 'display_sub3']

        self.main_camera = 1
        self._image_listeners = []
        self._image_publishers = []
        self._pub_img_msgs = [Image()] * len(_pub_topic_names)

        for idx, (camera_name, sub_topic_name) in enumerate(zip(_camera_names, _sub_topic_names)):
            self._image_listeners.append(ImageListener(camera_name, sub_topic_name, self._pub_img_msgs, idx, idx))

        for pub_topic_name in _pub_topic_names:
            self._image_publishers.append(rospy.Publisher(pub_topic_name, Image, queue_size=1))

    def _handle_image_switcher(self, req):

        if DEBUG: print('switch!')
        new_main_id = req.main_camera_id - 1
        new_main_order = 0
        for image_listener in self._image_listeners:
            if image_listener.id is new_main_id:
                new_main_order = image_listener.order
                image_listener.set_order(0)
                break

        cur_main_id = self.main_camera - 1
        for image_listener in self._image_listeners:
            if image_listener.id is cur_main_id:
                image_listener.set_order(new_main_order)
                break

        print(new_main_id, cur_main_id)

        self.main_camera = req.main_camera_id

    def activate(self):
        for image_listener in self._image_listeners:
            image_listener.activate()

        service = rospy.Service('image_switcher', ImageSwitcherService, self._handle_image_switcher)
        print 'READY...'
        # rospy.spin() # spin() is called in ros_looprate_manager.sleep()

    def pub_images(self):
        for image_pub, img_msg in zip(self._image_publishers, self._pub_img_msgs):
            image_pub.publish(img_msg)

    def deactivate(self):
        for image_listener in self._image_listeners:
            image_listener.deactivate()


if __name__ == '__main__':
    rospy.init_node('my_image_publisher', anonymous=True)

    image_switcher = ImageSwitcher()
    image_switcher.activate()

    ros_loop_mgr = rospy.Rate(30)  # hz
    while not rospy.is_shutdown():
        image_switcher.pub_images()
        ros_loop_mgr.sleep()

