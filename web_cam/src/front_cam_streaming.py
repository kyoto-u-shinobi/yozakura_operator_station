#!/usr/bin/env python

from image_streaming import WebCamManager

'''
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
http://atagan-memo.blogspot.jp/2012/11/opencvai-ballwifi.html
http://venuschjp.blogspot.jp/2015/02/pythonopencvweb.html
'''

DEBUG = True

AI_BALL_IP_FRONT = '192.168.54.161'

if __name__ == '__main__':
    web_cam_front = WebCamManager(AI_BALL_IP_FRONT, 'front_web_cam', 'front_cam_img')
    if DEBUG: print 'initialize!'
    if web_cam_front.open():
        if DEBUG: print 'run!'
        web_cam_front.run()
    else:
        print 'fail to open!'